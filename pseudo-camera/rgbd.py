from cv2 import INTER_NEAREST
import numpy as np
from tqdm import tqdm
import torch
from imageio import imread, imwrite
from path import Path
import os
import rospy
from cv_bridge import CvBridge
from sensor_msgs .msg import Image  as imageMsg
#from matplotlib import pyplot as plt
import cv2
from config import get_opts

from SC_Depth import SC_Depth
from SC_DepthV2 import SC_DepthV2

import datasets.custom_transforms as custom_transforms

from visualization import *
from PIL import Image as pimage
import matplotlib.pyplot as plt
# @torch.no_grad()
# def main():
#     hparams = get_opts()

#     if hparams.model_version == 'v1':
#         system = SC_Depth(hparams)
#     elif hparams.model_version == 'v2':
#         system = SC_DepthV2(hparams)

#     model = system.load_from_checkpoint(hparams.ckpt_path)
#     model.cuda()
#     model.eval()

#     # training size
#     if hparams.dataset_name == 'nyu':
#         training_size = [256, 320]
#     elif hparams.dataset_name == 'kitti':
#         training_size = [256, 832]
#     elif hparams.dataset_name == 'ddad':
#         training_size = [384, 640]

#     # normaliazation
#     inference_transform = custom_transforms.Compose([
#         custom_transforms.RescaleTo(training_size),
#         custom_transforms.ArrayToTensor(),
#         custom_transforms.Normalize()]
#     )

#     input_dir = Path(hparams.input_dir)
#     output_dir = Path(hparams.output_dir) / \
#         'model_{}'.format(hparams.model_version)
#     output_dir.makedirs_p()

#     if hparams.save_vis:
#         (output_dir/'vis').makedirs_p()

#     if hparams.save_depth:
#         (output_dir/'depth').makedirs_p()

#     image_files = sum([(input_dir).files('*.{}'.format(ext))
#                       for ext in ['jpg', 'png']], [])
#     image_files = sorted(image_files)

#     print('{} images for inference'.format(len(image_files)))

#     for i, img_file in enumerate(tqdm(image_files)):

#         filename = os.path.splitext(os.path.basename(img_file))[0]

#         img = imread(img_file).astype(np.float32)
#         tensor_img = inference_transform([img])[0][0].unsqueeze(0).cuda()
#         pred_depth = model.inference_depth(tensor_img)

#         if hparams.save_vis:
#             vis = visualize_depth(pred_depth[0, 0]).permute(
#                 1, 2, 0).numpy() * 255
#             imwrite(output_dir/'vis/{}.jpg'.format(filename),
#                     vis.astype(np.uint8))
@torch.no_grad()
def scipy_misc_imresize(arr, size, interp='bilinear', mode=None):
	im = pimage.fromarray(arr, mode=mode)
	ts = type(size)
	if np.issubdtype(ts, np.signedinteger):
		percent = size / 100.0
		size = tuple((np.array(im.size)*percent).astype(int))
	elif np.issubdtype(type(size), np.floating):
		size = tuple((np.array(im.size)*size).astype(int))
	else:
		size = (size[1], size[0])
	func = {'nearest': 0, 'lanczos': 1, 'bilinear': 2, 'bicubic': 3, 'cubic': 3}
	imnew = im.resize(size, resample=func[interp]) # 调用PIL库中的resize函数
	return np.array(imnew)


class PseudoCamera(object):


    @torch.no_grad()
    def __init__(self,hparams):
        #os.system(". /home/crescentrose/py3Ros_ws/devel/setup.sh")
        self.hparams = hparams
        fx = 924.873180
        fy = 923.504522
        cx = 486.997346
        cy = 364.308527
        self.intrinsic = torch.tensor(np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])).unsqueeze(0).to(torch.float32)           
        print(self.intrinsic.shape)
        print(self.intrinsic)
        self.hasInit = 0
        self.publish_prefix = 'pseudoCamera/'
        self.netDepth = 18
        self.cameraHeight = 720
        self.cameraWidth = 960
        self.netHeight = 256
        self.netWidth = 320
        self.targetHeight = 720
        self.targetWidth = 900
        self.bridge = CvBridge()
        self.training_size = [256, 320]

        self.system = SC_DepthV2(hparams)
        self.model = self.system.load_from_checkpoint(hparams.ckpt_path)
        self.model.cuda()
        self.model.eval()

        self.factor = 1000
        self.tooFar =  self.factor * 0.5
        self.inference_transform = custom_transforms.Compose([
        custom_transforms.RescaleTo(self.training_size),
        custom_transforms.ArrayToTensor(),
        custom_transforms.Normalize()]
    )

        # Initialize ROS
        rospy.loginfo("start pseudo camera init")
        rospy.init_node('pseudoCamera', anonymous=False) 
        rospy.loginfo('pseudoCamera is starting')
        rospy.Subscriber("/tello/camera/image_raw", imageMsg, self.callback)
        self.imgPub = rospy.Publisher(self.publish_prefix+'color',imageMsg,queue_size=10)
        self.depPub = rospy.Publisher(self.publish_prefix+'depth', imageMsg, queue_size=10)
        #self.posePub = rospy.Publisher(self.publish_prefix+'poseChange',)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

        """
        Listens for video streaming (raw h264) from the Tello.

        Runs as a thread, sets self.frame to the most recent frame Tello captured.

        """


    # def load_tensor_image(self,img):
    #     h, w, _ = img.shape
    #     if  (h != self.netHeight or w != self.netWidth):
    #         img = scipy_misc_imresize(img (self.netHeight, self.netWidth)).astype(np.float32)
    #     img = np.transpose(img, (2, 0, 1))
    #     tensor_img = ((torch.from_numpy(img).unsqueeze(0)/255-0.45)/0.225).to(self.device)
    #     return tensor_img

    def  depthCompensate(self,depth):
        depth = depth - 100/self.factor
        depth[np.where(depth<0)] = 0       
        k = 1
        depth[np.where(depth>0.5)] = 0
        depth = depth + k * np.power(depth,2.5)
        depth = depth * self.factor / 1.5
        #depth[np.where(depth>self.tooFar)] = 0
        
        depth[np.where(depth<100)] = 0       
        return depth


    @torch.no_grad()
    def callback(self,data):
      
        img = self.bridge.imgmsg_to_cv2(data,"bgr8")
        rawrawimg = img
        rawImg = np.uint8(img)
        img = img[0:720,30:930,0:3]
        rawImg = img
        rawImg = cv2.resize(rawImg,(320,256))
        h, w, _ = img.shape
        #img = imread(img_file).astype(np.float32)
        # if (h != self.netHeight or w != self.netWidth):
        #     img = scipy_misc_imresize(np.uint8(img), (self.netHeight, self.netWidth)).astype(np.float32)
        img = rawImg
        tensor_img = self.inference_transform([img])[0][0].unsqueeze(0).cuda()
        pred_depth = self.model.inference_depth(tensor_img)
        depth = pred_depth[0, 0].cpu().numpy()
        rawDepth = depth
        depth = self.depthCompensate(depth)
        # depth = depth * self.factor
        # depth[np.where(depth>self.tooFar)] = 0
        # depth = depth - 100
        # depth[np.where(depth<150)] = -1


        depthMap = np.zeros((720,960))
        depthMap[0:720,30:930] = cv2.resize(depth,[900,720],interpolation=INTER_NEAREST)
        depthMap[np.where(depthMap<150)] = 0
        depthMap[0:720,0:30] = 0
        depthMap[0:720,930:960] = 0
        dep_msg = self.bridge.cv2_to_imgmsg(depth)
        img_msg = self.bridge.cv2_to_imgmsg(img)

        dep_msg = self.bridge.cv2_to_imgmsg(depthMap)
        img_msg = self.bridge.cv2_to_imgmsg(rawrawimg)
        print("size: depth",depth.shape," img",img.shape)

        timeStamp =  rospy.Time.now()
        img_msg.header.stamp = timeStamp
        dep_msg.header.stamp = timeStamp
        self.imgPub.publish(img_msg)
        self.depPub.publish(dep_msg) 
        #print("after  cal  time:",time_af_ed - time_af_st)
        # visualArray = visualize_depth(pred_depth[0,0])
        # visualArray = np.transpose(visualArray.numpy(),(1,2,0))
        # visualArray2 =  cv2.resize(visualArray,(900,720))
        # print(visualArray.shape)
        # cv2.imshow('raw video',rawrawimg)
        # cv2.waitKey(1) 
        # cv2.imshow('depth video',visualArray2)
        # cv2.waitKey(1) 
        # cv2.imshow('origin',rawImg)
        # cv2.waitKey(1)
        #testMap = depthMap[]
        # print("avg depth is ",np.mean(depthMap)," max is ",np.max(depthMap)," min is ",np.min(depthMap),"size is",depthMap.shape)
        # plt.figure()
        # x = np.arange(0,255,1)
        # print(rawDepth.shape)
        # y = rawDepth[0:255,160]
        # plt.plot(x, y, linestyle='--', color='red')
        # plt.xlabel('x')
        # plt.ylabel('y')
        # ax = plt.gca()
        # ax.spines['right'].set_color('none')
        # ax.spines['top'].set_color('none')
        # ax.xaxis.set_ticks_position('bottom')
        # ax.yaxis.set_ticks_position('left')
        # ax.spines['bottom'].set_position(('data', 0))
        # ax.spines['left'].set_position(('data', 0))
        # plt.show()
        #os.system("pause")
        # cv2.imshow("auto mask ",auto_mask)
        # cv2.waitKey(1) 
        #self.lastFrame = tensor_img 
        #self.lastDepth =  origin_depth
        #self.last_tensor_img = tensor_img
        # cv2.imshow("valid mask ",valid_mask)
        # cv2.waitKey(1) 


if __name__ == '__main__':
    hparams = get_opts()
    PseudoCamera(hparams)
