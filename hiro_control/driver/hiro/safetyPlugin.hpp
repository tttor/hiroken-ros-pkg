// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.
#ifndef __safetyPlugin_hh__
#define __safetyPlugin_hh__

#ifndef __CORBA_H_EXTERNAL_GUARD__
#include <omniORB4/CORBA.h>
#endif

#ifndef  USE_stub_in_nt_dll
# define USE_stub_in_nt_dll_NOT_DEFINED_safetyPlugin
#endif
#ifndef  USE_core_stub_in_nt_dll
# define USE_core_stub_in_nt_dll_NOT_DEFINED_safetyPlugin
#endif
#ifndef  USE_dyn_stub_in_nt_dll
# define USE_dyn_stub_in_nt_dll_NOT_DEFINED_safetyPlugin
#endif



#ifndef __OpenHRPCommon_hh_EXTERNAL_GUARD__
#define __OpenHRPCommon_hh_EXTERNAL_GUARD__
#include <OpenHRPCommon.hpp>
#endif
#ifndef __HRPcontroller_hh_EXTERNAL_GUARD__
#define __HRPcontroller_hh_EXTERNAL_GUARD__
#include <HRPcontroller.hpp>
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





#ifndef __safetyPlugin__
#define __safetyPlugin__

class safetyPlugin;
class _objref_safetyPlugin;
class _impl_safetyPlugin;

typedef _objref_safetyPlugin* safetyPlugin_ptr;
typedef safetyPlugin_ptr safetyPluginRef;

class safetyPlugin_Helper {
public:
  typedef safetyPlugin_ptr _ptr_type;

  static _ptr_type _nil();
  static _CORBA_Boolean is_nil(_ptr_type);
  static void release(_ptr_type);
  static void duplicate(_ptr_type);
  static void marshalObjRef(_ptr_type, cdrStream&);
  static _ptr_type unmarshalObjRef(cdrStream&);
};

typedef _CORBA_ObjRef_Var<_objref_safetyPlugin, safetyPlugin_Helper> safetyPlugin_var;
typedef _CORBA_ObjRef_OUT_arg<_objref_safetyPlugin,safetyPlugin_Helper > safetyPlugin_out;

#endif

// interface safetyPlugin
class safetyPlugin {
public:
  // Declarations for this interface type.
  typedef safetyPlugin_ptr _ptr_type;
  typedef safetyPlugin_var _var_type;

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
  enum safetyModuleState { UnknownState, WakeUp, ActiveOperation, ProtectiveStop, eMergencyStop, ErrorState /*, __max_safetyModuleState=0xffffffff */ };
  typedef safetyModuleState& safetyModuleState_out;

  static _core_attr const char * versionStringIDL;

  static _core_attr const char * versionDateIDL;


};

class _objref_safetyPlugin :
  public virtual OpenHRP::_objref_Plugin
{
public:
  ::CORBA::Long setAutoSetting(::CORBA::Boolean reLock, ::CORBA::Long unLockTime);
  ::CORBA::Long getAutoSetting(::CORBA::Boolean& reLock, ::CORBA::Long& unLockTime);
  safetyPlugin::safetyModuleState transitionP2M();
  safetyPlugin::safetyModuleState transitionM2P();
  safetyPlugin::safetyModuleState transitionP2A();
  safetyPlugin::safetyModuleState transitionA2P();
  safetyPlugin::safetyModuleState transitionA2M();
  safetyPlugin::safetyModuleState transition2E();
  safetyPlugin::safetyModuleState transitionE2W();
  safetyPlugin::safetyModuleState queryState();
  ::CORBA::ULong queryWatchDogUpdate();
  ::CORBA::Long setValue(::CORBA::Short id, ::CORBA::Long value);
  ::CORBA::Long getValue(::CORBA::Short id, ::CORBA::Long& value);
  ::CORBA::ULongLong getShmFrameNumber();
  ::CORBA::ULongLong getDi();

  inline _objref_safetyPlugin()  { _PR_setobj(0); }  // nil
  _objref_safetyPlugin(omniIOR*, omniIdentity*);

protected:
  virtual ~_objref_safetyPlugin();

  
private:
  virtual void* _ptrToObjRef(const char*);

  _objref_safetyPlugin(const _objref_safetyPlugin&);
  _objref_safetyPlugin& operator = (const _objref_safetyPlugin&);
  // not implemented

  friend class safetyPlugin;
};

class _pof_safetyPlugin : public _OMNI_NS(proxyObjectFactory) {
public:
  inline _pof_safetyPlugin() : _OMNI_NS(proxyObjectFactory)(safetyPlugin::_PD_repoId) {}
  virtual ~_pof_safetyPlugin();

  virtual omniObjRef* newObjRef(omniIOR*,omniIdentity*);
  virtual _CORBA_Boolean is_a(const char*) const;
};

class _impl_safetyPlugin :
  public virtual OpenHRP::_impl_Plugin
{
public:
  virtual ~_impl_safetyPlugin();

  virtual ::CORBA::Long setAutoSetting(::CORBA::Boolean reLock, ::CORBA::Long unLockTime) = 0;
  virtual ::CORBA::Long getAutoSetting(::CORBA::Boolean& reLock, ::CORBA::Long& unLockTime) = 0;
  virtual safetyPlugin::safetyModuleState transitionP2M() = 0;
  virtual safetyPlugin::safetyModuleState transitionM2P() = 0;
  virtual safetyPlugin::safetyModuleState transitionP2A() = 0;
  virtual safetyPlugin::safetyModuleState transitionA2P() = 0;
  virtual safetyPlugin::safetyModuleState transitionA2M() = 0;
  virtual safetyPlugin::safetyModuleState transition2E() = 0;
  virtual safetyPlugin::safetyModuleState transitionE2W() = 0;
  virtual safetyPlugin::safetyModuleState queryState() = 0;
  virtual ::CORBA::ULong queryWatchDogUpdate() = 0;
  virtual ::CORBA::Long setValue(::CORBA::Short id, ::CORBA::Long value) = 0;
  virtual ::CORBA::Long getValue(::CORBA::Short id, ::CORBA::Long& value) = 0;
  virtual ::CORBA::ULongLong getShmFrameNumber() = 0;
  virtual ::CORBA::ULongLong getDi() = 0;
  
public:  // Really protected, workaround for xlC
  virtual _CORBA_Boolean _dispatch(omniCallHandle&);

private:
  virtual void* _ptrToInterface(const char*);
  virtual const char* _mostDerivedRepoId();
  
};




class POA_safetyPlugin :
  public virtual _impl_safetyPlugin,
  public virtual POA_OpenHRP::Plugin
{
public:
  virtual ~POA_safetyPlugin();

  inline ::safetyPlugin_ptr _this() {
    return (::safetyPlugin_ptr) _do_this(::safetyPlugin::_PD_repoId);
  }
};





template <class _omniT>
class POA_safetyPlugin_tie : public virtual POA_safetyPlugin
{
public:
  POA_safetyPlugin_tie(_omniT& t)
    : pd_obj(&t), pd_poa(0), pd_rel(0) {}
  POA_safetyPlugin_tie(_omniT& t, ::PortableServer::POA_ptr p)
    : pd_obj(&t), pd_poa(p), pd_rel(0) {}
  POA_safetyPlugin_tie(_omniT* t, _CORBA_Boolean r=1)
    : pd_obj(t), pd_poa(0), pd_rel(r) {}
  POA_safetyPlugin_tie(_omniT* t, ::PortableServer::POA_ptr p,_CORBA_Boolean r=1)
    : pd_obj(t), pd_poa(p), pd_rel(r) {}
  ~POA_safetyPlugin_tie() {
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

  ::CORBA::Long setAutoSetting(::CORBA::Boolean reLock, ::CORBA::Long unLockTime) { return pd_obj->setAutoSetting(reLock, unLockTime); }
  ::CORBA::Long getAutoSetting(::CORBA::Boolean& reLock, ::CORBA::Long& unLockTime) { return pd_obj->getAutoSetting(reLock, unLockTime); }
  safetyPlugin::safetyModuleState transitionP2M() { return pd_obj->transitionP2M(); }
  safetyPlugin::safetyModuleState transitionM2P() { return pd_obj->transitionM2P(); }
  safetyPlugin::safetyModuleState transitionP2A() { return pd_obj->transitionP2A(); }
  safetyPlugin::safetyModuleState transitionA2P() { return pd_obj->transitionA2P(); }
  safetyPlugin::safetyModuleState transitionA2M() { return pd_obj->transitionA2M(); }
  safetyPlugin::safetyModuleState transition2E() { return pd_obj->transition2E(); }
  safetyPlugin::safetyModuleState transitionE2W() { return pd_obj->transitionE2W(); }
  safetyPlugin::safetyModuleState queryState() { return pd_obj->queryState(); }
  ::CORBA::ULong queryWatchDogUpdate() { return pd_obj->queryWatchDogUpdate(); }
  ::CORBA::Long setValue(::CORBA::Short id, ::CORBA::Long value) { return pd_obj->setValue(id, value); }
  ::CORBA::Long getValue(::CORBA::Short id, ::CORBA::Long& value) { return pd_obj->getValue(id, value); }
  ::CORBA::ULongLong getShmFrameNumber() { return pd_obj->getShmFrameNumber(); }
  ::CORBA::ULongLong getDi() { return pd_obj->getDi(); }
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

inline void operator >>=(safetyPlugin::safetyModuleState _e, cdrStream& s) {
  ::operator>>=((::CORBA::ULong)_e, s);
}

inline void operator <<= (safetyPlugin::safetyModuleState& _e, cdrStream& s) {
  ::CORBA::ULong _0RL_e;
  ::operator<<=(_0RL_e,s);
  if (_0RL_e <= safetyPlugin::ErrorState) {
    _e = (safetyPlugin::safetyModuleState) _0RL_e;
  }
  else {
    OMNIORB_THROW(MARSHAL,_OMNI_NS(MARSHAL_InvalidEnumValue),
                  (::CORBA::CompletionStatus)s.completion());
  }
}



inline void
safetyPlugin::_marshalObjRef(::safetyPlugin_ptr obj, cdrStream& s) {
  omniObjRef::_marshal(obj->_PR_getobj(),s);
}




#ifdef   USE_stub_in_nt_dll_NOT_DEFINED_safetyPlugin
# undef  USE_stub_in_nt_dll
# undef  USE_stub_in_nt_dll_NOT_DEFINED_safetyPlugin
#endif
#ifdef   USE_core_stub_in_nt_dll_NOT_DEFINED_safetyPlugin
# undef  USE_core_stub_in_nt_dll
# undef  USE_core_stub_in_nt_dll_NOT_DEFINED_safetyPlugin
#endif
#ifdef   USE_dyn_stub_in_nt_dll_NOT_DEFINED_safetyPlugin
# undef  USE_dyn_stub_in_nt_dll
# undef  USE_dyn_stub_in_nt_dll_NOT_DEFINED_safetyPlugin
#endif

#endif  // __safetyPlugin_hh__

