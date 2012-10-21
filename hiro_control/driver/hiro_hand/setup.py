from VnetHand import *
from omniORB import CORBA
import CosNaming

def initCORBA(hostname, portno=2809):
    global Orb, Rootnc
    argv = [ '-ORBInitRef', 'NameService=corbaloc:iiop:%s:%d/NameService'%(hostname, portno) ]
    Orb = getORB(argv)
    ns = Orb.resolve_initial_references('NameService')
    Rootnc = ns._narrow(CosNaming.NamingContext)

def getORB(argv):
    orb = CORBA.ORB_init(argv, CORBA.ORB_ID)
    return orb

def findObject(objname):
    global Rootnc
    obj = None
    name = [CosNaming.NameComponent(objname, "")]
    try:
        obj = Rootnc.resolve(name)
    except CosNaming.NotFound:
        print 'exception in findObject (name:%s).'%( objname )
        raise
    return obj 

def find_vnet3Finger(name = 'vnet3Finger'):
    import sys
    obj = findObject(name)
    try:
        tmp = obj._narrow(vnet3Finger)
        return tmp
    except:
        sys.stderr.write('\n')
        sys.stderr.write('narrow exception: %s to %s\n'%( corbaObject, corbaClass ) )
        raise
    return None

def getObjRef(ip_addr='localhost'):
    initCORBA(ip_addr)
    v3f = find_vnet3Finger()
    return v3f
