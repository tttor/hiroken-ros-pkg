// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.
#ifndef __servoPlugin_hh__
#define __servoPlugin_hh__

#ifndef __CORBA_H_EXTERNAL_GUARD__
#include <omniORB4/CORBA.h>
#endif

#ifndef  USE_stub_in_nt_dll
# define USE_stub_in_nt_dll_NOT_DEFINED_servoPlugin
#endif
#ifndef  USE_core_stub_in_nt_dll
# define USE_core_stub_in_nt_dll_NOT_DEFINED_servoPlugin
#endif
#ifndef  USE_dyn_stub_in_nt_dll
# define USE_dyn_stub_in_nt_dll_NOT_DEFINED_servoPlugin
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





_CORBA_MODULE servoPluginTypes

_CORBA_MODULE_BEG

  class LngLngSeq_var;

  class LngLngSeq : public _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::LongLong, 8, 8 >  {
  public:
    typedef LngLngSeq_var _var_type;
    inline LngLngSeq() {}
    inline LngLngSeq(const LngLngSeq& _s)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::LongLong, 8, 8 > (_s) {}

    inline LngLngSeq(_CORBA_ULong _max)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::LongLong, 8, 8 > (_max) {}
    inline LngLngSeq(_CORBA_ULong _max, _CORBA_ULong _len, ::CORBA::LongLong* _val, _CORBA_Boolean _rel=0)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::LongLong, 8, 8 > (_max, _len, _val, _rel) {}

  

    inline LngLngSeq& operator = (const LngLngSeq& _s) {
      _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::LongLong, 8, 8 > ::operator=(_s);
      return *this;
    }
  };

  class LngLngSeq_out;

  class LngLngSeq_var {
  public:
    inline LngLngSeq_var() : _pd_seq(0) {}
    inline LngLngSeq_var(LngLngSeq* _s) : _pd_seq(_s) {}
    inline LngLngSeq_var(const LngLngSeq_var& _s) {
      if( _s._pd_seq )  _pd_seq = new LngLngSeq(*_s._pd_seq);
      else              _pd_seq = 0;
    }
    inline ~LngLngSeq_var() { if( _pd_seq )  delete _pd_seq; }
      
    inline LngLngSeq_var& operator = (LngLngSeq* _s) {
      if( _pd_seq )  delete _pd_seq;
      _pd_seq = _s;
      return *this;
    }
    inline LngLngSeq_var& operator = (const LngLngSeq_var& _s) {
      if( _s._pd_seq ) {
        if( !_pd_seq )  _pd_seq = new LngLngSeq;
        *_pd_seq = *_s._pd_seq;
      } else if( _pd_seq ) {
        delete _pd_seq;
        _pd_seq = 0;
      }
      return *this;
    }
    inline ::CORBA::LongLong& operator [] (_CORBA_ULong _s) {
      return (*_pd_seq)[_s];
    }

  

    inline LngLngSeq* operator -> () { return _pd_seq; }
    inline const LngLngSeq* operator -> () const { return _pd_seq; }
#if defined(__GNUG__)
    inline operator LngLngSeq& () const { return *_pd_seq; }
#else
    inline operator const LngLngSeq& () const { return *_pd_seq; }
    inline operator LngLngSeq& () { return *_pd_seq; }
#endif
      
    inline const LngLngSeq& in() const { return *_pd_seq; }
    inline LngLngSeq&       inout()    { return *_pd_seq; }
    inline LngLngSeq*&      out() {
      if( _pd_seq ) { delete _pd_seq; _pd_seq = 0; }
      return _pd_seq;
    }
    inline LngLngSeq* _retn() { LngLngSeq* tmp = _pd_seq; _pd_seq = 0; return tmp; }
      
    friend class LngLngSeq_out;
    
  private:
    LngLngSeq* _pd_seq;
  };

  class LngLngSeq_out {
  public:
    inline LngLngSeq_out(LngLngSeq*& _s) : _data(_s) { _data = 0; }
    inline LngLngSeq_out(LngLngSeq_var& _s)
      : _data(_s._pd_seq) { _s = (LngLngSeq*) 0; }
    inline LngLngSeq_out(const LngLngSeq_out& _s) : _data(_s._data) {}
    inline LngLngSeq_out& operator = (const LngLngSeq_out& _s) {
      _data = _s._data;
      return *this;
    }
    inline LngLngSeq_out& operator = (LngLngSeq* _s) {
      _data = _s;
      return *this;
    }
    inline operator LngLngSeq*&()  { return _data; }
    inline LngLngSeq*& ptr()       { return _data; }
    inline LngLngSeq* operator->() { return _data; }

    inline ::CORBA::LongLong& operator [] (_CORBA_ULong _i) {
      return (*_data)[_i];
    }

  

    LngLngSeq*& _data;

  private:
    LngLngSeq_out();
    LngLngSeq_out& operator=(const LngLngSeq_var&);
  };

  typedef ::CORBA::Float FltArray15[15];
  typedef ::CORBA::Float FltArray15_slice;

  _CORBA_MODULE_INLINE FltArray15_slice* FltArray15_alloc() {
    return new FltArray15_slice[15];
  }

  _CORBA_MODULE_INLINE FltArray15_slice* FltArray15_dup(const FltArray15_slice* _s) {
    if (!_s) return 0;
    FltArray15_slice* _data = FltArray15_alloc();
    if (_data) {
      for (_CORBA_ULong _0i0 = 0; _0i0 < 15; _0i0++){
        
        _data[_0i0] = _s[_0i0];

      }
  
    }
    return _data;
  }

  _CORBA_MODULE_INLINE void FltArray15_copy(FltArray15_slice* _to, const FltArray15_slice* _from){
    for (_CORBA_ULong _0i0 = 0; _0i0 < 15; _0i0++){
      
      _to[_0i0] = _from[_0i0];

    }
  
  }

  _CORBA_MODULE_INLINE void FltArray15_free(FltArray15_slice* _s) {
    delete [] _s;
  }

  class FltArray15_copyHelper {
  public:
    static inline FltArray15_slice* alloc() { return ::servoPluginTypes::FltArray15_alloc(); }
    static inline FltArray15_slice* dup(const FltArray15_slice* p) { return ::servoPluginTypes::FltArray15_dup(p); }
    static inline void free(FltArray15_slice* p) { ::servoPluginTypes::FltArray15_free(p); }
  };

  typedef _CORBA_Array_Fix_Var<FltArray15_copyHelper,FltArray15_slice> FltArray15_var;
  typedef _CORBA_Array_Fix_Forany<FltArray15_copyHelper,FltArray15_slice> FltArray15_forany;

  typedef FltArray15_slice* FltArray15_out;

  typedef ::CORBA::Boolean BolArray15[15];
  typedef ::CORBA::Boolean BolArray15_slice;

  _CORBA_MODULE_INLINE BolArray15_slice* BolArray15_alloc() {
    return new BolArray15_slice[15];
  }

  _CORBA_MODULE_INLINE BolArray15_slice* BolArray15_dup(const BolArray15_slice* _s) {
    if (!_s) return 0;
    BolArray15_slice* _data = BolArray15_alloc();
    if (_data) {
      for (_CORBA_ULong _0i0 = 0; _0i0 < 15; _0i0++){
        
        _data[_0i0] = _s[_0i0];

      }
  
    }
    return _data;
  }

  _CORBA_MODULE_INLINE void BolArray15_copy(BolArray15_slice* _to, const BolArray15_slice* _from){
    for (_CORBA_ULong _0i0 = 0; _0i0 < 15; _0i0++){
      
      _to[_0i0] = _from[_0i0];

    }
  
  }

  _CORBA_MODULE_INLINE void BolArray15_free(BolArray15_slice* _s) {
    delete [] _s;
  }

  class BolArray15_copyHelper {
  public:
    static inline BolArray15_slice* alloc() { return ::servoPluginTypes::BolArray15_alloc(); }
    static inline BolArray15_slice* dup(const BolArray15_slice* p) { return ::servoPluginTypes::BolArray15_dup(p); }
    static inline void free(BolArray15_slice* p) { ::servoPluginTypes::BolArray15_free(p); }
  };

  typedef _CORBA_Array_Fix_Var<BolArray15_copyHelper,BolArray15_slice> BolArray15_var;
  typedef _CORBA_Array_Fix_Forany<BolArray15_copyHelper,BolArray15_slice> BolArray15_forany;

  typedef BolArray15_slice* BolArray15_out;

  typedef ::CORBA::LongLong LngArray15[15];
  typedef ::CORBA::LongLong LngArray15_slice;

  _CORBA_MODULE_INLINE LngArray15_slice* LngArray15_alloc() {
    return new LngArray15_slice[15];
  }

  _CORBA_MODULE_INLINE LngArray15_slice* LngArray15_dup(const LngArray15_slice* _s) {
    if (!_s) return 0;
    LngArray15_slice* _data = LngArray15_alloc();
    if (_data) {
      for (_CORBA_ULong _0i0 = 0; _0i0 < 15; _0i0++){
        
        _data[_0i0] = _s[_0i0];

      }
  
    }
    return _data;
  }

  _CORBA_MODULE_INLINE void LngArray15_copy(LngArray15_slice* _to, const LngArray15_slice* _from){
    for (_CORBA_ULong _0i0 = 0; _0i0 < 15; _0i0++){
      
      _to[_0i0] = _from[_0i0];

    }
  
  }

  _CORBA_MODULE_INLINE void LngArray15_free(LngArray15_slice* _s) {
    delete [] _s;
  }

  class LngArray15_copyHelper {
  public:
    static inline LngArray15_slice* alloc() { return ::servoPluginTypes::LngArray15_alloc(); }
    static inline LngArray15_slice* dup(const LngArray15_slice* p) { return ::servoPluginTypes::LngArray15_dup(p); }
    static inline void free(LngArray15_slice* p) { ::servoPluginTypes::LngArray15_free(p); }
  };

  typedef _CORBA_Array_Fix_Var<LngArray15_copyHelper,LngArray15_slice> LngArray15_var;
  typedef _CORBA_Array_Fix_Forany<LngArray15_copyHelper,LngArray15_slice> LngArray15_forany;

  typedef LngArray15_slice* LngArray15_out;

_CORBA_MODULE_END

#ifndef __servoPlugin__
#define __servoPlugin__

class servoPlugin;
class _objref_servoPlugin;
class _impl_servoPlugin;

typedef _objref_servoPlugin* servoPlugin_ptr;
typedef servoPlugin_ptr servoPluginRef;

class servoPlugin_Helper {
public:
  typedef servoPlugin_ptr _ptr_type;

  static _ptr_type _nil();
  static _CORBA_Boolean is_nil(_ptr_type);
  static void release(_ptr_type);
  static void duplicate(_ptr_type);
  static void marshalObjRef(_ptr_type, cdrStream&);
  static _ptr_type unmarshalObjRef(cdrStream&);
};

typedef _CORBA_ObjRef_Var<_objref_servoPlugin, servoPlugin_Helper> servoPlugin_var;
typedef _CORBA_ObjRef_OUT_arg<_objref_servoPlugin,servoPlugin_Helper > servoPlugin_out;

#endif

// interface servoPlugin
class servoPlugin {
public:
  // Declarations for this interface type.
  typedef servoPlugin_ptr _ptr_type;
  typedef servoPlugin_var _var_type;

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

class _objref_servoPlugin :
  public virtual OpenHRP::_objref_Plugin
{
public:
  ::CORBA::Boolean switchServo(::CORBA::Boolean t);
  ::CORBA::Boolean jointCalibration(const ::servoPluginTypes::LngLngSeq& seq, const ::servoPluginTypes::FltArray15 angles, const ::servoPluginTypes::BolArray15 rot, ::hiroCommonStatus::statSequence_out status);
  ::CORBA::Boolean getStatus(::servoPluginTypes::LngArray15 status);
  ::CORBA::Boolean clearEmergency();
  ::CORBA::Boolean checkCalibration();

  inline _objref_servoPlugin()  { _PR_setobj(0); }  // nil
  _objref_servoPlugin(omniIOR*, omniIdentity*);

protected:
  virtual ~_objref_servoPlugin();

  
private:
  virtual void* _ptrToObjRef(const char*);

  _objref_servoPlugin(const _objref_servoPlugin&);
  _objref_servoPlugin& operator = (const _objref_servoPlugin&);
  // not implemented

  friend class servoPlugin;
};

class _pof_servoPlugin : public _OMNI_NS(proxyObjectFactory) {
public:
  inline _pof_servoPlugin() : _OMNI_NS(proxyObjectFactory)(servoPlugin::_PD_repoId) {}
  virtual ~_pof_servoPlugin();

  virtual omniObjRef* newObjRef(omniIOR*,omniIdentity*);
  virtual _CORBA_Boolean is_a(const char*) const;
};

class _impl_servoPlugin :
  public virtual OpenHRP::_impl_Plugin
{
public:
  virtual ~_impl_servoPlugin();

  virtual ::CORBA::Boolean switchServo(::CORBA::Boolean t) = 0;
  virtual ::CORBA::Boolean jointCalibration(const ::servoPluginTypes::LngLngSeq& seq, const ::servoPluginTypes::FltArray15 angles, const ::servoPluginTypes::BolArray15 rot, ::hiroCommonStatus::statSequence_out status) = 0;
  virtual ::CORBA::Boolean getStatus(::servoPluginTypes::LngArray15 status) = 0;
  virtual ::CORBA::Boolean clearEmergency() = 0;
  virtual ::CORBA::Boolean checkCalibration() = 0;
  
public:  // Really protected, workaround for xlC
  virtual _CORBA_Boolean _dispatch(omniCallHandle&);

private:
  virtual void* _ptrToInterface(const char*);
  virtual const char* _mostDerivedRepoId();
  
};




_CORBA_MODULE POA_servoPluginTypes
_CORBA_MODULE_BEG

_CORBA_MODULE_END

class POA_servoPlugin :
  public virtual _impl_servoPlugin,
  public virtual POA_OpenHRP::Plugin
{
public:
  virtual ~POA_servoPlugin();

  inline ::servoPlugin_ptr _this() {
    return (::servoPlugin_ptr) _do_this(::servoPlugin::_PD_repoId);
  }
};



_CORBA_MODULE OBV_servoPluginTypes
_CORBA_MODULE_BEG

_CORBA_MODULE_END



template <class _omniT>
class POA_servoPlugin_tie : public virtual POA_servoPlugin
{
public:
  POA_servoPlugin_tie(_omniT& t)
    : pd_obj(&t), pd_poa(0), pd_rel(0) {}
  POA_servoPlugin_tie(_omniT& t, ::PortableServer::POA_ptr p)
    : pd_obj(&t), pd_poa(p), pd_rel(0) {}
  POA_servoPlugin_tie(_omniT* t, _CORBA_Boolean r=1)
    : pd_obj(t), pd_poa(0), pd_rel(r) {}
  POA_servoPlugin_tie(_omniT* t, ::PortableServer::POA_ptr p,_CORBA_Boolean r=1)
    : pd_obj(t), pd_poa(p), pd_rel(r) {}
  ~POA_servoPlugin_tie() {
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

  ::CORBA::Boolean switchServo(::CORBA::Boolean t) { return pd_obj->switchServo(t); }
  ::CORBA::Boolean jointCalibration(const servoPluginTypes::LngLngSeq& seq, const servoPluginTypes::FltArray15 angles, const servoPluginTypes::BolArray15 rot, hiroCommonStatus::statSequence_out status) { return pd_obj->jointCalibration(seq, angles, rot, status); }
  ::CORBA::Boolean getStatus(servoPluginTypes::LngArray15 status) { return pd_obj->getStatus(status); }
  ::CORBA::Boolean clearEmergency() { return pd_obj->clearEmergency(); }
  ::CORBA::Boolean checkCalibration() { return pd_obj->checkCalibration(); }
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



inline void
servoPlugin::_marshalObjRef(::servoPlugin_ptr obj, cdrStream& s) {
  omniObjRef::_marshal(obj->_PR_getobj(),s);
}




#ifdef   USE_stub_in_nt_dll_NOT_DEFINED_servoPlugin
# undef  USE_stub_in_nt_dll
# undef  USE_stub_in_nt_dll_NOT_DEFINED_servoPlugin
#endif
#ifdef   USE_core_stub_in_nt_dll_NOT_DEFINED_servoPlugin
# undef  USE_core_stub_in_nt_dll
# undef  USE_core_stub_in_nt_dll_NOT_DEFINED_servoPlugin
#endif
#ifdef   USE_dyn_stub_in_nt_dll_NOT_DEFINED_servoPlugin
# undef  USE_dyn_stub_in_nt_dll
# undef  USE_dyn_stub_in_nt_dll_NOT_DEFINED_servoPlugin
#endif

#endif  // __servoPlugin_hh__

