#!/usr/bin/env python

from artifact_msgs.srv import GetNetvladDesc, GetNetvladDescResponse
import rospy
from netvlad.load_saved_model import load_saved_model
from cv_bridge import CvBridge
import torchvision.transforms as transforms
import os
import numpy as np
import torch

class GetNetvladDescServer():
    def __init__(self):
        dirname = os.path.dirname(os.path.realpath(__file__))
        saved_model_path = os.path.join(dirname, 'netvlad/weights/vgg16_netvlad_checkpoint/checkpoints/checkpoint.pth.tar')
        self.pca_components = np.load(dirname+'/netvlad/weights/pca_components.npy')[:rospy.get_param('~netvlad_desc_dim')].T
        self.model_loaded = True
        self.cuda = torch.cuda.is_available()
        try:
            self.model = load_saved_model(saved_model_path,cuda = self.cuda)
            if not self.cuda:
                print("CUDA not being used, running NetVLAD on CPU")
        except:
            print('Failed loading the weights to the NetVLAD model. Do the weights match the architecture?')
            self.model_loaded = False
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                std=[0.229, 0.224, 0.225]),
        ])
        self.bridge = CvBridge()
        s = rospy.Service('get_netvlad_desc', GetNetvladDesc, self.compute_descriptor)

        rospy.spin()

    def compute_descriptor(self, req):
        
        if not self.model_loaded:
            rospy.logwarn("NetVLAD service: Trying to compute a descriptor, but model was not loaded properly.")
            return GetNetvladDescResponse(list())

        if not req.im.data:
            rospy.logwarn("NetVLAD service: Received an empty image")
            return GetNetvladDescResponse(list())
        
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(req.im, desired_encoding='passthrough')
        except:
            rospy.logwarn("NetVLAD service: Received an invalid image")
            return GetNetvladDescResponse(list())

        # Check if 3 channels
        if len(cv_image.shape) != 3:
            rospy.logwarn("NetVLAD service: Received an image that does not have 3 channels")
            return GetNetvladDescResponse(list())

        if self.cuda:
            im_t = self.transform(cv_image).cuda().unsqueeze(0)
        else:
            im_t = self.transform(cv_image).unsqueeze(0)
        with torch.no_grad():
            desc = self.model.pool(self.model.encoder(im_t)).squeeze().cpu().numpy()
        desc = np.matmul(desc,self.pca_components)

        return GetNetvladDescResponse(list(desc))


if __name__ == "__main__":
    rospy.init_node('get_netvlad_desc_server')
    GetNetvladDescServer()
