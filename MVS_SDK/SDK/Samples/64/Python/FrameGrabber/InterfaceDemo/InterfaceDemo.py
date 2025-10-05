# -- coding: utf-8 --

import sys
import os
import termios

from ctypes import *

sys.path.append("../../MvImport")
from MvCameraControl_class import *

def press_any_key_exit():
    fd = sys.stdin.fileno()
    old_ttyinfo = termios.tcgetattr(fd)
    new_ttyinfo = old_ttyinfo[:]
    new_ttyinfo[3] &= ~termios.ICANON
    new_ttyinfo[3] &= ~termios.ECHO

    termios.tcsetattr(fd, termios.TCSANOW, new_ttyinfo)
    try:
        os.read(fd, 7)
    except:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, old_ttyinfo)


if __name__ == "__main__":

    # ch:初始化SDK | en: initialize SDK
    MvCamera.MV_CC_Initialize()

    interfaceList = MV_INTERFACE_INFO_LIST()
    transportLayerType = MV_GIGE_INTERFACE | MV_CAMERALINK_INTERFACE | MV_CXP_INTERFACE | MV_XOF_INTERFACE
    
    # ch:枚举采集卡 | en:Enum interfaces
    ret = MvCamera.MV_CC_EnumInterfaces(transportLayerType, interfaceList)
    if ret != 0:
        print("enum interfaces fail! ret[0x%x]" % ret)
        sys.exit()

    if interfaceList.nInterfaceNum == 0:
        print("find no interface!")
        sys.exit()

    print("Find %d interfaces!" % interfaceList.nInterfaceNum)

    for i in range(0, interfaceList.nInterfaceNum):
        interfaceInfo = cast(interfaceList.pInterfaceInfos[i], POINTER(MV_INTERFACE_INFO)).contents
        print("interface: [%d]" % i)
        
        displayName = ''.join([chr(c) for c in interfaceInfo.chDisplayName if c != 0])
        print("display name: %s" % displayName)

        serialNumber = ''.join([chr(c) for c in interfaceInfo.chSerialNumber if c != 0])
        print("serial number: %s" % serialNumber)

        modelName = ''.join([chr(c) for c in interfaceInfo.chModelName if c != 0])
        print("model name: %s" % modelName)

        interfaceId = ''.join([chr(c) for c in interfaceInfo.chInterfaceID if c != 0])
        print("interface id: %s" % interfaceId)

    nConnectionNum = input("please input the number of the interface to connect:")

    if int(nConnectionNum) >= interfaceList.nInterfaceNum:
        print("input error!")
        sys.exit()

    # ch:创建相机实例 | en:Create Camera Object
    cam = MvCamera()
    
    # ch:选择采集卡并创建句柄 | en:Select interface and create handle
    curInterface = cast(interfaceList.pInterfaceInfos[int(nConnectionNum)], POINTER(MV_INTERFACE_INFO)).contents

    ret = cam.MV_CC_CreateInterface(curInterface)
    if ret != 0:
        print("create interface handle fail! ret[0x%x]" % ret)
        sys.exit()

    # ch:打开设备 | en:Open device
    ret = cam.MV_CC_OpenInterface()
    if ret != 0:
        print("open interface fail! ret[0x%x]" % ret)
        sys.exit()
    else:
        print("open interface success")

    # ch:获取属性 | en:Get Feature
    stEnumValue = MVCC_ENUMVALUE()
    ret =cam.MV_CC_GetEnumValue("StreamSelector", stEnumValue)
    if ret != 0:
        print("get StreamSelector fail! ret[0x%x]" % ret)

    # ch:设置属性 | en:Set Feature
    ret = cam.MV_CC_SetEnumValue("StreamSelector", stEnumValue.nCurValue)
    if ret != 0:
        print("set StreamSelector fail! ret[0x%x]" % ret)
        sys.exit()
    else:
        print("set StreamSelector [%d] success" % stEnumValue.nCurValue)

    # ch:关闭采集卡 | en:Close interface
    ret = cam.MV_CC_CloseInterface()
    if ret != 0:
        print("close interface fail! ret[0x%x]" % ret)
        sys.exit()
    else:
        print("close interface success")

    # ch:销毁采集卡句柄 | en:Destroy interface
    ret = cam.MV_CC_DestroyInterface()
    if ret != 0:
        print("destroy interface fail! ret[0x%x]" % ret)
        sys.exit()

    print("press a key to exit.")
    press_any_key_exit()

    # ch:反初始化SDK | en: finalize SDK
    MvCamera.MV_CC_Finalize()