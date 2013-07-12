#ifndef LIBSVM_UTIL_INCLUDED
#define LIBSVM_UTIL_INCLUDED

#include "libsvm_util.hpp"
#include "svm.h"
#include "data.hpp"

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))

typedef struct svm_problem SVMProblem;
typedef struct svm_parameter SVMParameter;
typedef struct svm_model SVMModel;
typedef struct svm_node SVMNode;

static int (*info)(const char *fmt,...) = &printf;

// TODO unglobalize these vars
static char *line = NULL;
static int max_line_len;

//! Init svm params with the default values
/*!
  Note that these values are for SVM regression
  
  
*/
void
init_svmparam(SVMParameter* param)
{
  // default values
  param->svm_type = EPSILON_SVR;//EPSILON_SVR, NU_SVR
  param->kernel_type = RBF;
  param->degree = 3;
  param->gamma = 0;	// default: 1/num_features
  param->coef0 = 0;
  param->nu = 0.5;
  param->cache_size = 100;
  param->C = 1;
  param->eps = 1e-3;
  param->p = 0.1;
  param->shrinking = 1;
  param->probability = 0;
  param->nr_weight = 0;
  param->weight_label = NULL;
  param->weight = NULL;
}

bool
print_err_line(int line_num)
{
	fprintf(stderr,"Wrong input format at line %d\n", line_num);
	return false;
}

static char* 
readline(FILE *input)
{
	int len;
	
	if(fgets(line,max_line_len,input) == NULL)
		return NULL;

	while(strrchr(line,'\n') == NULL)
	{
		max_line_len *= 2;
		line = (char *) realloc(line,max_line_len);
		len = (int) strlen(line);
		if(fgets(line+len,max_line_len-len,input) == NULL)
			break;
	}
	return line;
}

//! read in a problem (in svmlight format)
bool
read_problem(const char *filename,SVMNode* x_space,SVMProblem* prob,SVMParameter* param)
{
	int elements, max_index, inst_max_index, i, j;
	FILE *fp = fopen(filename,"r");
	char *endptr;
	char *idx, *val, *label;

	if(fp == NULL)
	{
		fprintf(stderr,"can't open input file %s\n",filename);
		return false;
	}

	prob->l = 0;
	elements = 0;
  
	max_line_len = 1024;
	
	line = Malloc(char,max_line_len);
	
	while(readline(fp)!=NULL)
	{
		char *p = strtok(line," \t"); // label

		// features
		while(1)
		{
			p = strtok(NULL," \t");
			if(p == NULL || *p == '\n') // check '\n' as ' ' may be after the last feature
				break;
			++elements;
		}
		++elements;
		++prob->l;
	}
	rewind(fp);

	prob->y = Malloc(double,prob->l);
	prob->x = Malloc(struct svm_node *,prob->l);
	x_space = Malloc(struct svm_node,elements);

	max_index = 0;
	j=0;
	for(i=0;i<prob->l;i++)
	{
		inst_max_index = -1; // strtol gives 0 if wrong format, and precomputed kernel has <index> start from 0
		readline(fp);
		prob->x[i] = &x_space[j];
		label = strtok(line," \t\n");
		if(label == NULL) // empty line
			return print_err_line(i+1);

		prob->y[i] = strtod(label,&endptr);
		if(endptr == label || *endptr != '\0')
			return print_err_line(i+1);

		while(1)
		{
			idx = strtok(NULL,":");
			val = strtok(NULL," \t");

			if(val == NULL)
				break;

			errno = 0;
			x_space[j].index = (int) strtol(idx,&endptr,10);
			if(endptr == idx || errno != 0 || *endptr != '\0' || x_space[j].index <= inst_max_index)
				return print_err_line(i+1);
			else
				inst_max_index = x_space[j].index;

			errno = 0;
			x_space[j].value = strtod(val,&endptr);
			if(endptr == val || errno != 0 || (*endptr != '\0' && !isspace(*endptr)))
				return print_err_line(i+1);

			++j;
		}

		if(inst_max_index > max_index)
			max_index = inst_max_index;
		x_space[j++].index = -1;
	}

	if(param->gamma == 0 && max_index > 0)
		param->gamma = 1.0/max_index;

	if(param->kernel_type == PRECOMPUTED)
		for(i=0;i<prob->l;i++)
		{
			if (prob->x[i][0].index != 0)
			{
				fprintf(stderr,"Wrong input format: first column must be 0:sample_serial_number\n");
				return false;
			}
			if ((int)prob->x[i][0].value <= 0 || (int)prob->x[i][0].value > max_index)
			{
				fprintf(stderr,"Wrong input format: sample_serial_number out of range\n");
				return false;
			}
		}

	fclose(fp);
	
	return true;
}

double
libsvm_predict(SVMModel* model,int predict_probability,int max_n_attr_,SVMNode*x,FILE *input, FILE *output)
{
	int correct = 0;
	int total = 0;
	double error = 0;
	double sump = 0, sumt = 0, sumpp = 0, sumtt = 0, sumpt = 0;

	int svm_type=svm_get_svm_type(model);
	int nr_class=svm_get_nr_class(model);
	double *prob_estimates=NULL;
	int j;

	if(predict_probability)
	{
		if (svm_type==NU_SVR || svm_type==EPSILON_SVR)
			info("Prob. model for test data: target value = predicted value + z,\nz: Laplace distribution e^(-|z|/sigma)/(2sigma),sigma=%g\n",svm_get_svr_probability(model));
		else
		{
			int *labels=(int *) malloc(nr_class*sizeof(int));
			svm_get_labels(model,labels);
			prob_estimates = (double *) malloc(nr_class*sizeof(double));
			fprintf(output,"labels");		
			for(j=0;j<nr_class;j++)
				fprintf(output," %d",labels[j]);
			fprintf(output,"\n");
			free(labels);
		}
	}

	max_line_len = 1024;
	line = (char *)malloc(max_line_len*sizeof(char));
	while(readline(input) != NULL)
	{
		int i = 0;
		double target_label, predict_label;
		char *idx, *val, *label, *endptr;
		int inst_max_index = -1; // strtol gives 0 if wrong format, and precomputed kernel has <index> start from 0

		label = strtok(line," \t\n");
		if(label == NULL) // empty line
			print_err_line(total+1);

		target_label = strtod(label,&endptr);
		if(endptr == label || *endptr != '\0')
			print_err_line(total+1);

		while(1)
		{
			if(i>=max_n_attr_-1)	// need one more for index = -1
			{
				max_n_attr_ *= 2;
				x = (struct svm_node *) realloc(x,max_n_attr_*sizeof(struct svm_node));
			}

			idx = strtok(NULL,":");
			val = strtok(NULL," \t");

			if(val == NULL)
				break;
			errno = 0;
			x[i].index = (int) strtol(idx,&endptr,10);
			if(endptr == idx || errno != 0 || *endptr != '\0' || x[i].index <= inst_max_index)
				print_err_line(total+1);
			else
				inst_max_index = x[i].index;

			errno = 0;
			x[i].value = strtod(val,&endptr);
			if(endptr == val || errno != 0 || (*endptr != '\0' && !isspace(*endptr)))
				print_err_line(total+1);

			++i;
		}
		x[i].index = -1;

		if (predict_probability && (svm_type==C_SVC || svm_type==NU_SVC))
		{
			predict_label = svm_predict_probability(model,x,prob_estimates);
			fprintf(output,"%g",predict_label);
			for(j=0;j<nr_class;j++)
				fprintf(output," %g",prob_estimates[j]);
			fprintf(output,"\n");
		}
		else
		{
			predict_label = svm_predict(model,x);
			fprintf(output,"%g\n",predict_label);
		}

		if(predict_label == target_label)
			++correct;
		error += (predict_label-target_label)*(predict_label-target_label);
		sump += predict_label;
		sumt += target_label;
		sumpp += predict_label*predict_label;
		sumtt += target_label*target_label;
		sumpt += predict_label*target_label;
		++total;
	}
	if (svm_type==NU_SVR || svm_type==EPSILON_SVR)
	{
		info("Mean squared error = %g (regression)\n",error/total);
		info("Squared correlation coefficient = %g (regression)\n",
			((total*sumpt-sump*sumt)*(total*sumpt-sump*sumt))/
			((total*sumpp-sump*sump)*(total*sumtt-sumt*sumt))
			);
	}
	else
		info("Accuracy = %g%% (%d/%d) (classification)\n",
			(double)correct/total*100,correct,total);
	if(predict_probability)
		free(prob_estimates);
		
	return error/total;
}

class SVM_Object
{
public:
SVM_Object(std::string model_path,std::string te_data_path,std::string fit_path,std::string tr_data_path)
: model_path_(model_path),te_data_path_(te_data_path),fit_data_path_(fit_path),tr_data_path_(tr_data_path)
{ }

SVM_Object(SVMModel* model,int max_n_attr)
: model_(model), max_n_attr_(max_n_attr)
{ }

std::vector<double>
predict(const Input& in)
{
  std::vector<double> est_y;

  // Construct input x: convert Input& to SVMNode*
  SVMNode* x;
  x = (struct svm_node *) malloc(max_n_attr_*sizeof(struct svm_node));
  
  size_t idx = 0 ;
  for(Input::const_iterator i=in.begin(); i!=in.end(); ++i)
  {
    x[idx].index = (int)(i-in.begin()) + 1;// libsvm manual: <index> is an integer starting from 1 and _must_ be in Ascending order
    x[idx].value = *i;
    
    ++idx;
  }
  x[idx].index = -1;// index = -1 indicates the end of one vector

  // Predict
//  std::cerr << "svm_predict(model_,x) ..." << std::endl;
//  std::cerr << "svm_get_svm_type(model_)= " << svm_get_svm_type(model_) << std::endl;
  est_y.push_back( svm_predict(model_,x) );
  
  return est_y;
}

////! Predict 
///*!
//  Note that because libsvm only accept data in file-form, 
//  therefor we have to write input data into a file and read the resulted fitting value from a file.
//*/
//std::vector<double>
//predict(const Input& in)
//{
//  std::cerr << "SVM::predict(): BEGIN" << std::endl;
//  
//  if( !write_libsvm_data(in,te_data_path_) )
//  {
//    std::cerr << "can't write input file= " << te_data_path_ << std::endl;
//    return std::vector<double>();
//  }
//  std::cerr << "_in_ is written in te_data file" << std::endl;
//  
//  FILE *input;
//	input = fopen(te_data_path_.c_str(),"r");
//	if(input == NULL)
//	{
//    std::cerr << "can't open input file= " << te_data_path_ << std::endl;
//    return std::vector<double>();
//	}
//	std::cerr << "te_data is opened" << std::endl;
//  
//  FILE *output;
//	output = fopen(fit_data_path_.c_str(),"w");
//	if(output == NULL)
//	{
//    std::cerr << "can't open output file= " << fit_data_path_ << std::endl;
//    return std::vector<double>();
//	}
//	std::cerr << "out file is opened" << std::endl;
//	
//	SVMModel* model;
//	if( (model=svm_load_model(model_path_.c_str())) == 0 )
//	{
//    std::cerr << "can't open model file= " << model_path_ << std::endl;
//    return std::vector<double>();
//	}
//  std::cerr << "model file is opened" << std::endl;
//  
//  SVMNode* x;
//  int max_n_attr_ = 100;
//  
//  x = (struct svm_node *) malloc(max_n_attr_*sizeof(struct svm_node));

//  int predict_probability=0;
//  
//  std::cerr << "libsvm_predict()..." << std::endl;
//  libsvm_predict(model,predict_probability,max_n_attr_,x,input,output);

//  // TODO make these work, now: causes running-time ERR    
////  svm_free_and_destroy_model(&model);
////  free(x);
////  free(line);// NOTE that line is a global var
//  fclose(input);
//  fclose(output);
//  
//  return get_fit_vals();
//}

//! This is only a dummy function: SVR does not update.
std::vector<double>
update(const std::vector<double>& x,const std::vector<double>& y)
{
  std::cerr << "This is only a dummy function: SVR does not update." << std::endl;
  
  return std::vector<double>();
}

//! This is only a dummy function so far.
bool
writeBinary(const char* 	filename)
{
  return false;
}

//! This is only a dummy function so far.
size_t
nData()
{
  return 0;
}

private:
//! Obtaining fitted values for _one_ single instance from a file 
std::vector<double>
get_fit_vals()
{
  std::vector<double> fit_vals;
  
  //Read Only the first line
  std::ifstream fit_in(fit_data_path_.c_str());
  
  if(fit_in.is_open())
  {
    std::string fit_val;
    getline(fit_in,fit_val);
    
    if(fit_val.size() != 0)
      fit_vals.push_back( boost::lexical_cast<double>(fit_val) );
  }
  fit_in.close();
  
  return fit_vals;
}

std::string model_path_;
std::string te_data_path_;
std::string fit_data_path_;
std::string tr_data_path_;

SVMModel* model_;
int max_n_attr_;
};

#endif // #ifndef LIBSVM_UTIL_INCLUDED
