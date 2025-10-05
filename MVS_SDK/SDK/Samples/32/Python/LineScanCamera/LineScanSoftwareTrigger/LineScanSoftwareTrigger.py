# -- coding: utf-8 --

import sys
import os
import threading
import termios
import time
sys.path.append("../../MvImport")
from MvCameraControl_class import *

g_exit = False
winfun_ctype = CFUNCTYPE

stFrameInfo = POINTER(MV_FRAME_OUT_INFO_EX)
pData = POINTER(c_ubyte)
FrameInfoCallBack = winfun_ctype(None, pData, stFrameInfo, c_void_p)


def image_callback(pData, pFrameInfo, pUser):
        stFrameInfo = cast(pFrameInfo, POINTER(MV_FRAME_OUT_INFO_EX)).contents
        if stFrameInfo:
            print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.nFrameNum))


CALL_BACK_FUN = FrameInfoCallBack(image_callback)

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


# 发送软触发命令 | en: send software trigger command
def software_trigger_thread(cam_ins=0, cmd_value=""):
    while True:
        res = cam.MV_CC_SetCommandValue(cmd_value)
        if res != 0:
            print("set software trigger command fail[0x%x]" % res)
        time.sleep(1)
        if g_exit is True:
            break


def check_feature_node_access(cam_ins, node_name):
    access_mode = MV_XML_AccessMode()
    res = cam_ins.MV_XML_GetNodeAccessMode(node_name, access_mode)
    if res != 0:
        return False
    if access_mode == AM_WO or access_mode == AM_RO or access_mode == AM_RW:
        return True
    else:
        return False


if __name__ == "__main__":

    try:
        # ch:初始化SDK | en: initialize SDK
        MvCamera.MV_CC_Initialize()

        deviceList = MV_CC_DEVICE_INFO_LIST()
        tlayerType = (MV_GIGE_DEVICE | MV_USB_DEVICE)

        # ch:枚举设备 | en:Enum device
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
        if ret != 0:
            print("enum devices fail! ret[0x%x]" % ret)
            sys.exit()

        if deviceList.nDeviceNum == 0:
            print("find no device!")
            sys.exit()

        print("Find %d devices!" % deviceList.nDeviceNum)

        for i in range(0, deviceList.nDeviceNum):
            mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE or mvcc_dev_info.nTLayerType == MV_GENTL_GIGE_DEVICE:
                print ("\ngige device: [%d]" % i)
                strModeName = ''.join([chr(c) for c in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName if c != 0])
                print ("device model name: %s" % strModeName)

                nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
                nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
                nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
                nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
                print("current ip: %d.%d.%d.%d\n" % (nip1, nip2, nip3, nip4))
            elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
                print ("\nu3v device: [%d]" % i)
                strModeName = ''.join([chr(c) for c in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName if c != 0])
                print ("device model name: %s" % strModeName)

                strSerialNumber =  ''.join([chr(c) for c in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber if c != 0])                
                print ("user serial number: %s" % strSerialNumber)


        nConnectionNum = input("please input the number of the device to connect:")

        if int(nConnectionNum) >= deviceList.nDeviceNum:
            print("input error!")
            sys.exit()

        # ch:创建相机实例 | en:Create Camera Object
        cam = MvCamera()

        # ch:选择设备并创建句柄 | en:Select device and create handle
        stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents

        ret = cam.MV_CC_CreateHandle(stDeviceList)
        if ret != 0:
            raise Exception("create handle fail! ret[0x%x]" % ret)

        # ch:打开设备 | en:Open device
        ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            raise Exception("open device fail! ret[0x%x]" % ret)

        # ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if stDeviceList.nTLayerType == MV_GIGE_DEVICE or stDeviceList.nTLayerType == MV_GENTL_GIGE_DEVICE:
            nPacketSize = cam.MV_CC_GetOptimalPacketSize()
            if int(nPacketSize) > 0:
                ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)
                if ret != 0:
                    print("Warning: Set Packet Size fail! ret[0x%x]" % ret)
            else:
                print("Warning: Get Packet Size fail! ret[0x%x]" % nPacketSize)

        # ch:线阵相机帧触发预设置 | en:Settings in FrameScan mode
        ret = cam.MV_CC_SetEnumValueByString("ScanMode", "FrameScan")
        if ret == 0:
            print("set frame scan mode")

        # ch: 判断FrameTriggerControl是否可读 | en: Check if FrameTriggerControl is readable
        if check_feature_node_access(cam, "FrameTriggerControl"):
            # ch:设置触发模式为on | en:Set trigger mode as on
            ret = cam.MV_CC_SetBoolValue("FrameTriggerMode", True)
            if ret != 0:
                raise Exception("set frame trigger mode on fail! ret[0x%x]" % ret)
            # ch:设置触发源为Line0 | en:Set trigger source as Line0
            ret = cam.MV_CC_SetEnumValueByString("FrameTriggerSource", "Software")
            if ret != 0:
                raise Exception("set trigger source Software fail! ret[0x%x]" % ret)

            trigger_cmd = "FrameTriggerSoftware"
        else:
            # ch:设置触发选项为FrameBurstStart | en:Set trigger selector as FrameBurstStart
            ret = cam.MV_CC_SetEnumValue("TriggerSelector", 6)
            if ret != 0:
                raise Exception("set trigger selector fail! ret[0x%x]" % ret)

            # ch:设置触发模式为on | en:Set trigger mode as on
            ret = cam.MV_CC_SetEnumValue("TriggerMode", 1)
            if ret != 0:
                raise Exception("set trigger mode fail! ret[0x%x]" % ret)

            # ch:设置触发源为Line0 | en:Set trigger source as Line0
            ret = cam.MV_CC_SetEnumValueByString("TriggerSource", "Software")
            if ret != 0:
                raise Exception("set trigger source fail! ret[0x%x]" % ret)

            trigger_cmd = "TriggerSoftware"

        # ch:注册抓图回调 | en:Register image callback
        ret = cam.MV_CC_RegisterImageCallBackEx(CALL_BACK_FUN, None)
        if ret != 0:
            raise Exception("register image callback fail! ret[0x%x]" % ret)

        # ch:开始取流 | en:Start grab image
        ret = cam.MV_CC_StartGrabbing()
        if ret != 0:
            raise Exception("start grabbing fail! ret[0x%x]" % ret)

        try:
            hThreadHandle = threading.Thread(target=software_trigger_thread, args=(cam, trigger_cmd))
            hThreadHandle.start()
        except:
            print("error: unable to start thread")

        print("press a key to stop grabbing.")
        press_any_key_exit()
        g_exit = True
        hThreadHandle.join()

        # ch:停止取流 | en:Stop grab image
        ret = cam.MV_CC_StopGrabbing()
        if ret != 0:
            raise Exception("stop grabbing fail! ret[0x%x]" % ret)

        # ch:关闭设备 | Close device
        ret = cam.MV_CC_CloseDevice()
        if ret != 0:
            raise Exception("close device fail! ret[0x%x]" % ret)

        # ch:销毁句柄 | Destroy handle0
        cam.MV_CC_DestroyHandle()

    except Exception as e:
        print(e)
        cam.MV_CC_CloseDevice()
        cam.MV_CC_DestroyHandle()
    finally:
        # ch:反初始化SDK | en: finalize SDK
        MvCamera.MV_CC_Finalize()
