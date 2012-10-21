// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.
#ifndef __statusPlugin_hh__
#define __statusPlugin_hh__

#ifndef __CORBA_H_EXTERNAL_GUARD__
#include <omniORB4/CORBA.h>
#endif

#ifndef  USE_stub_in_nt_dll
# define USE_stub_in_nt_dll_NOT_DEFINED_statusPlugin
#endif
#ifndef  USE_core_stub_in_nt_dll
# define USE_core_stub_in_nt_dll_NOT_DEFINED_statusPlugin
#endif
#ifndef  USE_dyn_stub_in_nt_dll
# define USE_dyn_stub_in_nt_dll_NOT_DEFINED_statusPlugin
#endif



#ifndef __OpenHRPCommon_hh_EXTERNAL_GUARD__
#define __OpenHRPCommon_hh_EXTERNAL_GUARD__
#include <OpenHRPCommon.hpp>
#endif
#ifndef __HRPcontroller_hh_EXTERNAL_GUARD__
#define __HRPcontroller_hh_EXTERNAL_GUARD__
#include <HRPcontroller.hpp>
#endif
#ifndef __hiroCommonStatus_hh_EXTERNAL_GUARD__
#define __hiroCommonStatus_hh_EXTERNAL_GUARD__
#include <hiroCommonStatus.hpp>
#endif



#ifdef USE_stub_in_nt_dll
# ifndef USE_core_stub_in_nt_dll
#  define USE_core_stub_in_nt_dll
# endif
# ifndef USE_dyn_stub_in_nt_dll
#  define USE_dyn_stub_in_nt_dll
# endif
#endif

#ifdef _core_attr
# error "A local CPP macro _core_attr has already been defined."
#else
# ifdef  USE_core_stub_in_nt_dll
#  define _core_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _core_attr
# endif
#endif

#ifdef _dyn_attr
# error "A local CPP macro _dyn_attr has already been defined."
#else
# ifdef  USE_dyn_stub_in_nt_dll
#  define _dyn_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _dyn_attr
# endif
#endif





_CORBA_MODULE statusPluginTypes

_CORBA_MODULE_BEG

  enum statusElement { SYSTEM_STATUS, CALIB_STATUS, SERVO_STATUS, EMERGENCY_STATUS, EMERBUTTON_STATUS, PROTECT_STATUS, MOTORHEATUP_STATUS, FANSTOP_STATUS, MAX_ELEMENT /*, __max_statusElement=0xffffffff */ };
  typedef statusElement& statusElement_out;

  typedef ::CORBA::Octet partType;
  typedef ::CORBA::Octet_out partType_out;

  typedef ::CORBA::ULongLong jointType;
  typedef ::CORBA::ULongLong_out jointType_out;

  typedef ::CORBA::UShort kindType;
  typedef ::CORBA::UShort_out kindType_out;

  typedef ::CORBA::ULong robotStatusType;
  typedef ::CORBA::ULong_out robotStatusType_out;

  _CORBA_MODULE_VARINT const ::CORBA::Octet PART_CHEST _init_in_decl_( = 1 );

  _CORBA_MODULE_VARINT const ::CORBA::Octet PART_NECK _init_in_decl_( = 2 );

  _CORBA_MODULE_VARINT const ::CORBA::Octet PART_RARM _init_in_decl_( = 4 );

  _CORBA_MODULE_VARINT const ::CORBA::Octet PART_LARM _init_in_decl_( = 8 );

  _CORBA_MODULE_VARINT const ::CORBA::Octet PART_ALL _init_in_decl_( = 15 );

  _CORBA_MODULE_VARINT const ::CORBA::ULongLong JOINT_CHEST _init_in_decl_( = _CORBA_LONGLONG_CONST(1U) );

  _CORBA_MODULE_VARINT const ::CORBA::ULongLong JOINT_NECK _init_in_decl_( = _CORBA_LONGLONG_CONST(6U) );

  _CORBA_MODULE_VARINT const ::CORBA::ULongLong JOINT_RARM _init_in_decl_( = _CORBA_LONGLONG_CONST(504U) );

  _CORBA_MODULE_VARINT const ::CORBA::ULongLong JOINT_LARM _init_in_decl_( = _CORBA_LONGLONG_CONST(32256U) );

  _CORBA_MODULE_VARINT const ::CORBA::ULongLong JOINT_ALL _init_in_decl_( = _CORBA_LONGLONG_CONST(32767U) );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_CHEST _init_in_decl_( = 0 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_NECK _init_in_decl_( = 1 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_RARM _init_in_decl_( = 2 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_LARM _init_in_decl_( = 3 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_PART _init_in_decl_( = 7 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_ANGLE _init_in_decl_( = 0 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_EULER _init_in_decl_( = 8 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_QUATER _init_in_decl_( = 16 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_MATRIX _init_in_decl_( = 24 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_EXPRESS _init_in_decl_( = 56 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_COMMAND _init_in_decl_( = 0 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_STATUS _init_in_decl_( = 64 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_COM_STAT _init_in_decl_( = 192 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_MM_DEG _init_in_decl_( = 0 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_M_RAD _init_in_decl_( = 256 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_DIMENSION _init_in_decl_( = 768 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_LIM_UP _init_in_decl_( = 1024 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_LIM_DOWN _init_in_decl_( = 2048 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_LIM_ACC _init_in_decl_( = 3072 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_LIM_VEL _init_in_decl_( = 4096 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_LIM_DEC _init_in_decl_( = 5120 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_MC_WRITE _init_in_decl_( = 15360 );

  _CORBA_MODULE_VARINT const ::CORBA::UShort K_VALUE_TYPE _init_in_decl_( = 15360 );

  _CORBA_MODULE_VARINT const ::CORBA::ULong SYSTEM_ON _init_in_decl_( = 1U );

  _CORBA_MODULE_VARINT const ::CORBA::ULong CALIB_YET _init_in_decl_( = 16U );

  _CORBA_MODULE_VARINT const ::CORBA::ULong CALIB_DONE _init_in_decl_( = 32U );

  _CORBA_MODULE_VARINT const ::CORBA::ULong SERVO_ON _init_in_decl_( = 256U );

  _CORBA_MODULE_VARINT const ::CORBA::ULong SERVO_OFF _init_in_decl_( = 512U );

  _CORBA_MODULE_VARINT const ::CORBA::ULong EMERGENCY_CLEAR _init_in_decl_( = 4096U );

  _CORBA_MODULE_VARINT const ::CORBA::ULong SERVO_EMERGENCY _init_in_decl_( = 8192U );

  _CORBA_MODULE_VARINT const ::CORBA::ULong SERVO_READY _init_in_decl_( = 65536U );

  _CORBA_MODULE_VARINT const ::CORBA::ULong EMER_OR_BUTTON _init_in_decl_( = 131072U );

  _CORBA_MODULE_VARINT const ::CORBA::ULong EMER_AND_BUTTON _init_in_decl_( = 262144U );

  _CORBA_MODULE_VARINT const ::CORBA::ULong PROTECT_RELEASE _init_in_decl_( = 1048576U );

  _CORBA_MODULE_VARINT const ::CORBA::ULong PROTECT_STOP _init_in_decl_( = 2097152U );

  _CORBA_MODULE_VARINT const ::CORBA::ULong PROTECT_PARTS _init_in_decl_( = 4194304U );

  _CORBA_MODULE_VARINT const ::CORBA::ULong MOTOR_NORMAL _init_in_decl_( = 16777216U );

  _CORBA_MODULE_VARINT const ::CORBA::ULong MOTOR_HEATUP _init_in_decl_( = 33554432U );

  _CORBA_MODULE_VARINT const ::CORBA::ULong FAN_MOVE _init_in_decl_( = 268435456U );

  _CORBA_MODULE_VARINT const ::CORBA::ULong FAN_STOP _init_in_decl_( = 536870912U );

  class JointValues_var;

  class JointValues : public _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::Double, 8, 8 >  {
  public:
    typedef JointValues_var _var_type;
    inline JointValues() {}
    inline JointValues(const JointValues& _s)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::Double, 8, 8 > (_s) {}

    inline JointValues(_CORBA_ULong _max)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::Double, 8, 8 > (_max) {}
    inline JointValues(_CORBA_ULong _max, _CORBA_ULong _len, ::CORBA::Double* _val, _CORBA_Boolean _rel=0)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::Double, 8, 8 > (_max, _len, _val, _rel) {}

  

    inline JointValues& operator = (const JointValues& _s) {
      _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::Double, 8, 8 > ::operator=(_s);
      return *this;
    }
  };

  class JointValues_out;

  class JointValues_var {
  public:
    inline JointValues_var() : _pd_seq(0) {}
    inline JointValues_var(JointValues* _s) : _pd_seq(_s) {}
    inline JointValues_var(const JointValues_var& _s) {
      if( _s._pd_seq )  _pd_seq = new JointValues(*_s._pd_seq);
      else              _pd_seq = 0;
    }
    inline ~JointValues_var() { if( _pd_seq )  delete _pd_seq; }
      
    inline JointValues_var& operator = (JointValues* _s) {
      if( _pd_seq )  delete _pd_seq;
      _pd_seq = _s;
      return *this;
    }
    inline JointValues_var& operator = (const JointValues_var& _s) {
      if( _s._pd_seq ) {
        if( !_pd_seq )  _pd_seq = new JointValues;
        *_pd_seq = *_s._pd_seq;
      } else if( _pd_seq ) {
        delete _pd_seq;
        _pd_seq = 0;
      }
      return *this;
    }
    inline ::CORBA::Double& operator [] (_CORBA_ULong _s) {
      return (*_pd_seq)[_s];
    }

  

    inline JointValues* operator -> () { return _pd_seq; }
    inline const JointValues* operator -> () const { return _pd_seq; }
#if defined(__GNUG__)
    inline operator JointValues& () const { return *_pd_seq; }
#else
    inline operator const JointValues& () const { return *_pd_seq; }
    inline operator JointValues& () { return *_pd_seq; }
#endif
      
    inline const JointValues& in() const { return *_pd_seq; }
    inline JointValues&       inout()    { return *_pd_seq; }
    inline JointValues*&      out() {
      if( _pd_seq ) { delete _pd_seq; _pd_seq = 0; }
      return _pd_seq;
    }
    inline JointValues* _retn() { JointValues* tmp = _pd_seq; _pd_seq = 0; return tmp; }
      
    friend class JointValues_out;
    
  private:
    JointValues* _pd_seq;
  };

  class JointValues_out {
  public:
    inline JointValues_out(JointValues*& _s) : _data(_s) { _data = 0; }
    inline JointValues_out(JointValues_var& _s)
      : _data(_s._pd_seq) { _s = (JointValues*) 0; }
    inline JointValues_out(const JointValues_out& _s) : _data(_s._data) {}
    inline JointValues_out& operator = (const JointValues_out& _s) {
      _data = _s._data;
      return *this;
    }
    inline JointValues_out& operator = (JointValues* _s) {
      _data = _s;
      return *this;
    }
    inline operator JointValues*&()  { return _data; }
    inline JointValues*& ptr()       { return _data; }
    inline JointValues* operator->() { return _data; }

    inline ::CORBA::Double& operator [] (_CORBA_ULong _i) {
      return (*_data)[_i];
    }

  

    JointValues*& _data;

  private:
    JointValues_out();
    JointValues_out& operator=(const JointValues_var&);
  };

  class RobotStatus_var;

  class RobotStatus : public _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::ULong, 4, 4 >  {
  public:
    typedef RobotStatus_var _var_type;
    inline RobotStatus() {}
    inline RobotStatus(const RobotStatus& _s)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::ULong, 4, 4 > (_s) {}

    inline RobotStatus(_CORBA_ULong _max)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::ULong, 4, 4 > (_max) {}
    inline RobotStatus(_CORBA_ULong _max, _CORBA_ULong _len, ::CORBA::ULong* _val, _CORBA_Boolean _rel=0)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::ULong, 4, 4 > (_max, _len, _val, _rel) {}

  

    inline RobotStatus& operator = (const RobotStatus& _s) {
      _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::ULong, 4, 4 > ::operator=(_s);
      return *this;
    }
  };

  class RobotStatus_out;

  class RobotStatus_var {
  public:
    inline RobotStatus_var() : _pd_seq(0) {}
    inline RobotStatus_var(RobotStatus* _s) : _pd_seq(_s) {}
    inline RobotStatus_var(const RobotStatus_var& _s) {
      if( _s._pd_seq )  _pd_seq = new RobotStatus(*_s._pd_seq);
      else              _pd_seq = 0;
    }
    inline ~RobotStatus_var() { if( _pd_seq )  delete _pd_seq; }
      
    inline RobotStatus_var& operator = (RobotStatus* _s) {
      if( _pd_seq )  delete _pd_seq;
      _pd_seq = _s;
      return *this;
    }
    inline RobotStatus_var& operator = (const RobotStatus_var& _s) {
      if( _s._pd_seq ) {
        if( !_pd_seq )  _pd_seq = new RobotStatus;
        *_pd_seq = *_s._pd_seq;
      } else if( _pd_seq ) {
        delete _pd_seq;
        _pd_seq = 0;
      }
      return *this;
    }
    inline ::CORBA::ULong& operator [] (_CORBA_ULong _s) {
      return (*_pd_seq)[_s];
    }

  

    inline RobotStatus* operator -> () { return _pd_seq; }
    inline const RobotStatus* operator -> () const { return _pd_seq; }
#if defined(__GNUG__)
    inline operator RobotStatus& () const { return *_pd_seq; }
#else
    inline operator const RobotStatus& () const { return *_pd_seq; }
    inline operator RobotStatus& () { return *_pd_seq; }
#endif
      
    inline const RobotStatus& in() const { return *_pd_seq; }
    inline RobotStatus&       inout()    { return *_pd_seq; }
    inline RobotStatus*&      out() {
      if( _pd_seq ) { delete _pd_seq; _pd_seq = 0; }
      return _pd_seq;
    }
    inline RobotStatus* _retn() { RobotStatus* tmp = _pd_seq; _pd_seq = 0; return tmp; }
      
    friend class RobotStatus_out;
    
  private:
    RobotStatus* _pd_seq;
  };

  class RobotStatus_out {
  public:
    inline RobotStatus_out(RobotStatus*& _s) : _data(_s) { _data = 0; }
    inline RobotStatus_out(RobotStatus_var& _s)
      : _data(_s._pd_seq) { _s = (RobotStatus*) 0; }
    inline RobotStatus_out(const RobotStatus_out& _s) : _data(_s._data) {}
    inline RobotStatus_out& operator = (const RobotStatus_out& _s) {
      _data = _s._data;
      return *this;
    }
    inline RobotStatus_out& operator = (RobotStatus* _s) {
      _data = _s;
      return *this;
    }
    inline operator RobotStatus*&()  { return _data; }
    inline RobotStatus*& ptr()       { return _data; }
    inline RobotStatus* operator->() { return _data; }

    inline ::CORBA::ULong& operator [] (_CORBA_ULong _i) {
      return (*_data)[_i];
    }

  

    RobotStatus*& _data;

  private:
    RobotStatus_out();
    RobotStatus_out& operator=(const RobotStatus_var&);
  };

_CORBA_MODULE_END

#ifndef __statusPlugin__
#define __statusPlugin__

class statusPlugin;
class _objref_statusPlugin;
class _impl_statusPlugin;

typedef _objref_statusPlugin* statusPlugin_ptr;
typedef statusPlugin_ptr statusPluginRef;

class statusPlugin_Helper {
public:
  typedef statusPlugin_ptr _ptr_type;

  static _ptr_type _nil();
  static _CORBA_Boolean is_nil(_ptr_type);
  static void release(_ptr_type);
  static void duplicate(_ptr_type);
  static void marshalObjRef(_ptr_type, cdrStream&);
  static _ptr_type unmarshalObjRef(cdrStream&);
};

typedef _CORBA_ObjRef_Var<_objref_statusPlugin, statusPlugin_Helper> statusPlugin_var;
typedef _CORBA_ObjRef_OUT_arg<_objref_statusPlugin,statusPlugin_Helper > statusPlugin_out;

#endif

// interface statusPlugin
class statusPlugin {
public:
  // Declarations for this interface type.
  typedef statusPlugin_ptr _ptr_type;
  typedef statusPlugin_var _var_type;

  static _ptr_type _duplicate(_ptr_type);
  static _ptr_type _narrow(::CORBA::Object_ptr);
  static _ptr_type _unchecked_narrow(::CORBA::Object_ptr);
  
  static _ptr_type _nil();

  static inline void _marshalObjRef(_ptr_type, cdrStream&);

  static inline _ptr_type _unmarshalObjRef(cdrStream& s) {
    omniObjRef* o = omniObjRef::_unMarshal(_PD_repoId,s);
    if (o)
      return (_ptr_type) o->_ptrToObjRef(_PD_repoId);
    else
      return _nil();
  }

  static _core_attr const char* _PD_repoId;

  // Other IDL defined within this scope.
  
};

class _objref_statusPlugin :
  public virtual OpenHRP::_objref_Plugin
{
public:
  ::CORBA::Boolean inMotionJoint(::statusPluginTypes::jointType& joint, ::CORBA::Boolean wait, ::hiroCommonStatus::statSequence_out status);
  ::CORBA::Boolean inMotionPart(::statusPluginTypes::partType& part, ::CORBA::Boolean wait, ::hiroCommonStatus::statSequence_out status);
  ::CORBA::Boolean getJointValue(::CORBA::Octet startJoint, ::CORBA::Octet jointNum, ::statusPluginTypes::kindType outKind, ::statusPluginTypes::JointValues_out joint_values, ::hiroCommonStatus::statSequence_out status);
  ::CORBA::Boolean getPositionArm(::statusPluginTypes::kindType outKind, ::statusPluginTypes::JointValues_out joint_values, ::hiroCommonStatus::statSequence_out status);
  ::CORBA::Boolean getChangeMcName(::CORBA::Octet jointNum, ::CORBA::Double& mcID, ::CORBA::String_out name, ::hiroCommonStatus::statSequence_out status);
  ::CORBA::Boolean convertData(const ::statusPluginTypes::JointValues& inValues, const ::statusPluginTypes::JointValues& handOffset, ::statusPluginTypes::kindType inKind, ::statusPluginTypes::kindType outKind, ::statusPluginTypes::JointValues_out outValues, ::hiroCommonStatus::statSequence_out status);
  ::CORBA::Boolean setHandOffset(const ::statusPluginTypes::JointValues& handOffset, ::statusPluginTypes::kindType inKind, ::hiroCommonStatus::statSequence_out status);
  ::CORBA::Boolean getHandOffset(::statusPluginTypes::kindType outKind, ::statusPluginTypes::JointValues_out handOffset, ::hiroCommonStatus::statSequence_out status);
  void getStatus(::statusPluginTypes::RobotStatus_out robotStatuses);

  inline _objref_statusPlugin()  { _PR_setobj(0); }  // nil
  _objref_statusPlugin(omniIOR*, omniIdentity*);

protected:
  virtual ~_objref_statusPlugin();

  
private:
  virtual void* _ptrToObjRef(const char*);

  _objref_statusPlugin(const _objref_statusPlugin&);
  _objref_statusPlugin& operator = (const _objref_statusPlugin&);
  // not implemented

  friend class statusPlugin;
};

class _pof_statusPlugin : public _OMNI_NS(proxyObjectFactory) {
public:
  inline _pof_statusPlugin() : _OMNI_NS(proxyObjectFactory)(statusPlugin::_PD_repoId) {}
  virtual ~_pof_statusPlugin();

  virtual omniObjRef* newObjRef(omniIOR*,omniIdentity*);
  virtual _CORBA_Boolean is_a(const char*) const;
};

class _impl_statusPlugin :
  public virtual OpenHRP::_impl_Plugin
{
public:
  virtual ~_impl_statusPlugin();

  virtual ::CORBA::Boolean inMotionJoint(::statusPluginTypes::jointType& joint, ::CORBA::Boolean wait, ::hiroCommonStatus::statSequence_out status) = 0;
  virtual ::CORBA::Boolean inMotionPart(::statusPluginTypes::partType& part, ::CORBA::Boolean wait, ::hiroCommonStatus::statSequence_out status) = 0;
  virtual ::CORBA::Boolean getJointValue(::CORBA::Octet startJoint, ::CORBA::Octet jointNum, ::statusPluginTypes::kindType outKind, ::statusPluginTypes::JointValues_out joint_values, ::hiroCommonStatus::statSequence_out status) = 0;
  virtual ::CORBA::Boolean getPositionArm(::statusPluginTypes::kindType outKind, ::statusPluginTypes::JointValues_out joint_values, ::hiroCommonStatus::statSequence_out status) = 0;
  virtual ::CORBA::Boolean getChangeMcName(::CORBA::Octet jointNum, ::CORBA::Double& mcID, ::CORBA::String_out name, ::hiroCommonStatus::statSequence_out status) = 0;
  virtual ::CORBA::Boolean convertData(const ::statusPluginTypes::JointValues& inValues, const ::statusPluginTypes::JointValues& handOffset, ::statusPluginTypes::kindType inKind, ::statusPluginTypes::kindType outKind, ::statusPluginTypes::JointValues_out outValues, ::hiroCommonStatus::statSequence_out status) = 0;
  virtual ::CORBA::Boolean setHandOffset(const ::statusPluginTypes::JointValues& handOffset, ::statusPluginTypes::kindType inKind, ::hiroCommonStatus::statSequence_out status) = 0;
  virtual ::CORBA::Boolean getHandOffset(::statusPluginTypes::kindType outKind, ::statusPluginTypes::JointValues_out handOffset, ::hiroCommonStatus::statSequence_out status) = 0;
  virtual void getStatus(::statusPluginTypes::RobotStatus_out robotStatuses) = 0;
  
public:  // Really protected, workaround for xlC
  virtual _CORBA_Boolean _dispatch(omniCallHandle&);

private:
  virtual void* _ptrToInterface(const char*);
  virtual const char* _mostDerivedRepoId();
  
};




_CORBA_MODULE POA_statusPluginTypes
_CORBA_MODULE_BEG

_CORBA_MODULE_END

class POA_statusPlugin :
  public virtual _impl_statusPlugin,
  public virtual POA_OpenHRP::Plugin
{
public:
  virtual ~POA_statusPlugin();

  inline ::statusPlugin_ptr _this() {
    return (::statusPlugin_ptr) _do_this(::statusPlugin::_PD_repoId);
  }
};



_CORBA_MODULE OBV_statusPluginTypes
_CORBA_MODULE_BEG

_CORBA_MODULE_END



template <class _omniT>
class POA_statusPlugin_tie : public virtual POA_statusPlugin
{
public:
  POA_statusPlugin_tie(_omniT& t)
    : pd_obj(&t), pd_poa(0), pd_rel(0) {}
  POA_statusPlugin_tie(_omniT& t, ::PortableServer::POA_ptr p)
    : pd_obj(&t), pd_poa(p), pd_rel(0) {}
  POA_statusPlugin_tie(_omniT* t, _CORBA_Boolean r=1)
    : pd_obj(t), pd_poa(0), pd_rel(r) {}
  POA_statusPlugin_tie(_omniT* t, ::PortableServer::POA_ptr p,_CORBA_Boolean r=1)
    : pd_obj(t), pd_poa(p), pd_rel(r) {}
  ~POA_statusPlugin_tie() {
    if( pd_poa )  ::CORBA::release(pd_poa);
    if( pd_rel )  delete pd_obj;
  }

  _omniT* _tied_object() { return pd_obj; }

  void _tied_object(_omniT& t) {
    if( pd_rel )  delete pd_obj;
    pd_obj = &t;
    pd_rel = 0;
  }

  void _tied_object(_omniT* t, _CORBA_Boolean r=1) {
    if( pd_rel )  delete pd_obj;
    pd_obj = t;
    pd_rel = r;
  }

  _CORBA_Boolean _is_owner()        { return pd_rel; }
  void _is_owner(_CORBA_Boolean io) { pd_rel = io;   }

  ::PortableServer::POA_ptr _default_POA() {
    if( !pd_poa )  return ::PortableServer::POA::_the_root_poa();
    else           return ::PortableServer::POA::_duplicate(pd_poa);
  }

  ::CORBA::Boolean inMotionJoint(statusPluginTypes::jointType& joint, ::CORBA::Boolean wait, hiroCommonStatus::statSequence_out status) { return pd_obj->inMotionJoint(joint, wait, status); }
  ::CORBA::Boolean inMotionPart(statusPluginTypes::partType& part, ::CORBA::Boolean wait, hiroCommonStatus::statSequence_out status) { return pd_obj->inMotionPart(part, wait, status); }
  ::CORBA::Boolean getJointValue(::CORBA::Octet startJoint, ::CORBA::Octet jointNum, statusPluginTypes::kindType outKind, statusPluginTypes::JointValues_out joint_values, hiroCommonStatus::statSequence_out status) { return pd_obj->getJointValue(startJoint, jointNum, outKind, joint_values, status); }
  ::CORBA::Boolean getPositionArm(statusPluginTypes::kindType outKind, statusPluginTypes::JointValues_out joint_values, hiroCommonStatus::statSequence_out status) { return pd_obj->getPositionArm(outKind, joint_values, status); }
  ::CORBA::Boolean getChangeMcName(::CORBA::Octet jointNum, ::CORBA::Double& mcID, ::CORBA::String_out name, hiroCommonStatus::statSequence_out status) { return pd_obj->getChangeMcName(jointNum, mcID, name, status); }
  ::CORBA::Boolean convertData(const statusPluginTypes::JointValues& inValues, const statusPluginTypes::JointValues& handOffset, statusPluginTypes::kindType inKind, statusPluginTypes::kindType outKind, statusPluginTypes::JointValues_out outValues, hiroCommonStatus::statSequence_out status) { return pd_obj->convertData(inValues, handOffset, inKind, outKind, outValues, status); }
  ::CORBA::Boolean setHandOffset(const statusPluginTypes::JointValues& handOffset, statusPluginTypes::kindType inKind, hiroCommonStatus::statSequence_out status) { return pd_obj->setHandOffset(handOffset, inKind, status); }
  ::CORBA::Boolean getHandOffset(statusPluginTypes::kindType outKind, statusPluginTypes::JointValues_out handOffset, hiroCommonStatus::statSequence_out status) { return pd_obj->getHandOffset(outKind, handOffset, status); }
  void getStatus(statusPluginTypes::RobotStatus_out robotStatuses) { pd_obj->getStatus(robotStatuses); }
  void start() { pd_obj->start(); }
  void stop() { pd_obj->stop(); }
  void sendMsg(const char* msg) { pd_obj->sendMsg(msg); }


private:
  _omniT*                   pd_obj;
  ::PortableServer::POA_ptr pd_poa;
  _CORBA_Boolean            pd_rel;
};



#undef _core_attr
#undef _dyn_attr

inline void operator >>=(statusPluginTypes::statusElement _e, cdrStream& s) {
  ::operator>>=((::CORBA::ULong)_e, s);
}

inline void operator <<= (statusPluginTypes::statusElement& _e, cdrStream& s) {
  ::CORBA::ULong _0RL_e;
  ::operator<<=(_0RL_e,s);
  if (_0RL_e <= statusPluginTypes::MAX_ELEMENT) {
    _e = (statusPluginTypes::statusElement) _0RL_e;
  }
  else {
    OMNIORB_THROW(MARSHAL,_OMNI_NS(MARSHAL_InvalidEnumValue),
                  (::CORBA::CompletionStatus)s.completion());
  }
}



inline void
statusPlugin::_marshalObjRef(::statusPlugin_ptr obj, cdrStream& s) {
  omniObjRef::_marshal(obj->_PR_getobj(),s);
}




#ifdef   USE_stub_in_nt_dll_NOT_DEFINED_statusPlugin
# undef  USE_stub_in_nt_dll
# undef  USE_stub_in_nt_dll_NOT_DEFINED_statusPlugin
#endif
#ifdef   USE_core_stub_in_nt_dll_NOT_DEFINED_statusPlugin
# undef  USE_core_stub_in_nt_dll
# undef  USE_core_stub_in_nt_dll_NOT_DEFINED_statusPlugin
#endif
#ifdef   USE_dyn_stub_in_nt_dll_NOT_DEFINED_statusPlugin
# undef  USE_dyn_stub_in_nt_dll
# undef  USE_dyn_stub_in_nt_dll_NOT_DEFINED_statusPlugin
#endif

#endif  // __statusPlugin_hh__

