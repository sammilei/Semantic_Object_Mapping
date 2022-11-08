"""
An example that uses TensorRT's Python api to make inferences.
"""
import os
import shutil
import random
import sys
import threading
import time
import cv2
import numpy as np
import pycuda.autoinit
import pycuda.driver as cuda
import tensorrt as trt

from PIL import Image

# CONF_THRESH = 0.05
# IOU_THRESHOLD = 0.4
cuda.init()


def get_img_path_batches(batch_size, img_dir):
    ret = []
    batch = []
    for root, dirs, files in os.walk(img_dir):
        for name in files:
            if len(batch) == batch_size:
                ret.append(batch)
                batch = []
            batch.append(os.path.join(root, name))
    if len(batch) > 0:
        ret.append(batch)
    return ret


def plot_boxes(_x, img, color=None, line_thickness=None):
    """
    description: Plots one bounding box on image img,
                 this function comes from YoLov5 project.
    param: 
        x:      a box likes [x1,y1,x2,y2]
        img:    a opencv image object
        color:  color to draw rectangle, such as (0,255,0)
        label:  str
        line_thickness: int
    return:
        no return

    """
    img = np.moveaxis(img, 0, 2) * 255
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    for x in _x:
        label = str(int(x[5]))
        label = label + ": {:0.1%}".format(x[4])
        tl = (line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1)  # line/font thickness
        color = list(np.random.random(size=3) * 255)
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv2.rectangle(img, c1, c2, color, thickness=int(tl), lineType=cv2.LINE_AA)

        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=int(tl / 3), thickness=int(tf))[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(
            img,
            label,
            (c1[0], c1[1] - 2),
            0,
            tl / 3,
            [225, 255, 255],
            thickness=int(tf),
            lineType=cv2.LINE_AA,
        )
    return img


def resize_img(image, size):
    iw, ih = image.size
    w, h = size
    scale = min(w / iw, h / ih)
    nw = int(iw * scale)
    nh = int(ih * scale)

    image = image.resize((nw, nh), Image.BICUBIC)
    new_image = Image.new('RGB', size, (128, 128, 128))
    new_image.paste(image, ((w - nw) // 2, (h - nh) // 2))
    return new_image, scale, (nw, nh)


def resize_img_no_PIL(image, expected_size):
    ih, iw, _ = image.shape
    eh, ew = expected_size
    scale = min(eh / ih, ew / iw)
    nh = int(ih * scale)
    nw = int(iw * scale)

    image = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
    new_img = np.full((eh, ew, 3), 128, dtype='uint8')
    # fill new image with the resized image and centered it
    new_img[(eh - nh) // 2:(eh - nh) // 2 + nh,
          (ew - nw) // 2:(ew - nw) // 2 + nw,
          :] = image.copy()

    return new_img, scale, (nh, nw)


def clip_coords(boxes, img_shape):
    # Clip bounding xyxy bounding boxes to image shape (height, width)
    boxes[:, 0].clamp_(0, img_shape[1])  # x1
    boxes[:, 1].clamp_(0, img_shape[0])  # y1
    boxes[:, 2].clamp_(0, img_shape[1])  # x2
    boxes[:, 3].clamp_(0, img_shape[0])  # y2


def scale_coords(img1_shape, coords, img0_shape, ratio_pad=None):
    # Rescale coords (xyxy) from img1_shape to img0_shape
    if ratio_pad is None:  # calculate from img0_shape
        gain = max(img1_shape) / max(img0_shape)  # gain  = old / new
        pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2  # wh padding
    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]

    coords[:, [0, 2]] -= pad[0]  # x padding
    coords[:, [1, 3]] -= pad[1]  # y padding
    coords[:, :4] /= gain
    clip_coords(coords, img0_shape)
    return coords


def xywh2xyxy(x):
    # Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
    # y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
    y = np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y


def non_max_suppression(prediction, conf_thres=0.25, iou_thres=0.45, max_det=300):
    """Runs Non-Maximum Suppression (NMS) on inference results

    Returns:
         list of detections, on (n,6) tensor per image [xyxy, conf, cls]
    """
    nc = prediction.shape[2] - 5  # number of classes
    xc = prediction[..., 4] > conf_thres  # candidates

    # Checks
    assert 0 <= conf_thres <= 1  # , f'Invalid Confidence threshold {conf_thres}, valid values are between 0.0 and 1.0'
    assert 0 <= iou_thres <= 1  # , f'Invalid IoU {iou_thres}, valid values are between 0.0 and 1.0'

    # Settings
    min_wh, max_wh = 2, 4096  # (pixels) minimum and maximum box width and height
    max_nms = 3000  # maximum number of boxes into torchvision.ops.nms()
    time_limit = 10.0  # seconds to quit after

    t = time.time()
    # output = [torch.zeros((0, 6), device=prediction.device)] * prediction.shape[0]
    output = [np.zeros((0, 6))] * prediction.shape[0]
    for xi, x in enumerate(prediction):  # image index, image inference
        # Apply constraints
        x = x[xc[xi]]  # confidence
        # If none remain process next image
        if not x.shape[0]:
            continue

        # Compute conf
        x[:, 5:] *= x[:, 4:5]  # conf = obj_conf * cls_conf

        # Box (center x, center y, width, height) to (x1, y1, x2, y2)
        box = xywh2xyxy(x[:, :4])

        conf = x[:, 5:].max(1, keepdims=True)
        j = x[:, 5:].argmax(1)
        x = np.concatenate((box, conf, np.expand_dims(j, 1)), 1)[conf.reshape(-1) > conf_thres]

        # Check shape
        n = x.shape[0]  # number of boxes
        if not n:  # no boxes
            continue

        elif n > max_nms:  # excess boxes
            x = x[x[:, 4].argsort(descending=True)[:max_nms]]  # sort by confidence

        # Batched NMS
        c = x[:, 5:6] * (max_wh)  # classes
        boxes, scores = x[:, :4] + c, x[:, 4]  # boxes (offset by class), scores

        i = nms(boxes, scores, iou_thres)  # NMS

        if i.shape[0] > max_det:  # limit detections
            i = i[:max_det]

        output[xi] = np.array(x[i])
        if (time.time() - t) > time_limit:
            print('WARNING: NMS time limit {}s exceeded'.format(time_limit))
            break  # time limit exceeded

    return output


def nms(dets, scores, thresh):
    '''
    dets is a numpy array : num_dets, 4
    scores ia  nump array : num_dets,
    '''
    x1 = dets[:, 0]
    y1 = dets[:, 1]
    x2 = dets[:, 2]
    y2 = dets[:, 3]

    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = scores.argsort()[::-1]  # get boxes with more ious first

    keep = []
    while order.size > 0:
        i = order[0]  # pick maxmum iou box
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1 + 1)  # maximum width
        h = np.maximum(0.0, yy2 - yy1 + 1)  # maxiumum height
        inter = w * h
        ovr = inter / (areas[i] + areas[order[1:]] - inter)

        inds = np.where(ovr <= thresh)[0]
        order = order[inds + 1]

    return np.array(keep)


def time_synchronized():
    return time.time()


class TRTmodel(object):
    """
    description: A YOLOv5 class that warps TensorRT ops, preprocess and postprocess ops.
    """

    def __init__(self, engine_file_path, iou_thresh, conf_thresh):
        # Create a Context on this device,
        self.profile = False
        self.fps = []
        self.ctx = cuda.Device(0).make_context()
        stream = cuda.Stream()
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        runtime = trt.Runtime(TRT_LOGGER)

        # Deserialize the engine from file
        with open(engine_file_path, "rb") as f:
            engine = runtime.deserialize_cuda_engine(f.read())
        context = engine.create_execution_context()
        context.active_optimization_profile = 0

        host_inputs = []
        cuda_inputs = []
        host_outputs = []
        cuda_outputs = []
        bindings = []
        self.batch_size = engine.max_batch_size
        # self.batch_size = 5
        for idx, binding in enumerate(engine):
            print('binding:', binding, engine.get_binding_shape(binding))
            size = trt.volume(engine.get_binding_shape(binding))
            dtype = trt.nptype(engine.get_binding_dtype(binding))
            # Allocate host and device buffers
            host_mem = cuda.pagelocked_empty(size, dtype)
            cuda_mem = cuda.mem_alloc(host_mem.nbytes)
            # Append the device buffer to device bindings.
            bindings.append(int(cuda_mem))
            # Append to the appropriate list.
            if engine.binding_is_input(binding):
                self.img_size = engine.get_binding_shape(binding)[2:]
                host_inputs.append(host_mem)
                cuda_inputs.append(cuda_mem)

            else:
                self.out_size = engine.get_binding_shape(binding)
                host_outputs.append(host_mem)
                cuda_outputs.append(cuda_mem)

        # Store
        self.stream = stream
        self.context = context
        self.engine = engine
        self.host_inputs = host_inputs
        self.cuda_inputs = cuda_inputs
        self.host_outputs = host_outputs
        self.cuda_outputs = cuda_outputs
        self.bindings = bindings

        self.IOU_THRESHOLD = iou_thresh
        self.CONF_THRESH = conf_thresh

    def get_dim(self):
        return self.img_size[0], self.img_size[1]

    def run(self, batch):
        #threading.Thread.__init__(self)

        # Make self the active context, pushing it on top of the context stack.
        self.ctx.push()
        # Restore
        stream = self.stream
        context = self.context
        engine = self.engine
        host_inputs = self.host_inputs
        cuda_inputs = self.cuda_inputs
        host_outputs = self.host_outputs
        cuda_outputs = self.cuda_outputs
        bindings = self.bindings

        # Transfer input data to the GPU.

        np.copyto(host_inputs[0], batch.ravel())
        # np.copyto(host_inputs[0], batch.flat)
        cuda.memcpy_htod_async(cuda_inputs[0], host_inputs[0], stream)

        t0 = time_synchronized()
        # Run inference.
        context.execute_async_v2(bindings=bindings, stream_handle=stream.handle)
        # Transfer predictions back from the GPU.
        cuda.memcpy_dtoh_async(host_outputs[0], cuda_outputs[0], stream)
        # Synchronize the stream
        stream.synchronize()
        t1 = time_synchronized()
        # Remove any context from the top of the context stack, deactivating it.
        self.ctx.pop()

        output = host_outputs[0]
        # output = torch.from_numpy(np.array(output))
        output = np.array(output)
        # output = output.view(self.batch_size, -1, self.out_size[-1])
        output = output.reshape(self.batch_size, -1, self.out_size[-1])

        # Do postprocess
        output = non_max_suppression(output, conf_thres=self.CONF_THRESH, iou_thres=self.IOU_THRESHOLD)
        t2 = time_synchronized()

        if self.profile:
            print('inference time {}, nms time: {}, total: {} fps: {}'.format(round(1000 * (t1 - t0), 3), round((t2 - t1) * 1000, 3), round(1000 * (t2 - t0), 3), round(1 / (t2 - t0), 3)))
            self.fps.append(1 / (t2 - t0))
        return output

    def destroy(self):
        # Remove any context from the top of the context stack, deactivating it.
        self.ctx.pop()


class OnnxModel():
    def __init__(self, path):
        import onnxruntime

        so = onnxruntime.SessionOptions()
        so.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL

        self.session = onnxruntime.InferenceSession(path, sess_options=so)
        self.session.set_providers(['CUDAExecutionProvider'])

        self.profile = True
        self.batch_size = self.session.get_inputs()[0].shape[0]
        self.out_size = self.session.get_outputs()[0].shape
        self.fps = []

    def run(self, input):
        threading.Thread.__init__(self)
        t0 = time_synchronized()
        ort_inputs = {self.session.get_inputs()[0].name: np.array(input)}
        output = self.session.run(None, ort_inputs)
        t1 = time_synchronized()

        output = torch.from_numpy(np.array(output))
        output = output.view(self.batch_size, -1, self.out_size[2])
        output = non_max_suppression(output, conf_thres=CONF_THRESH, iou_thres=IOU_THRESHOLD)
        t2 = time_synchronized()

        if self.profile:
            print('inference time {}, nms time: {}, total: {} fps: {}'.format(round(1000 * (t1 - t0), 3), round((t2 - t1) * 1000, 3), round(1000 * (t2 - t0), 3), round(1 / (t2 - t0), 3)))
            self.fps.append(1 / (t2 - t0))
        return output

    def destroy(self):
        pass


class inferThread_using_batch_path(threading.Thread):
    def __init__(self, yolov5_wrapper, image_path_batch):
        #threading.Thread.__init__(self)
        self.yolov5_wrapper = yolov5_wrapper
        self.image_path_batch = image_path_batch

    def run(self):

        batch = []
        for image_path in self.image_path_batch:
            image_src = Image.open(image_path)
            resized = resize_img(image_src, (self.yolov5_wrapper.img_size[1], self.yolov5_wrapper.img_size[0]))
            img_in = np.transpose(resized, (2, 0, 1)).astype(np.float32)  # HWC -> CHW
            img_in = np.expand_dims(img_in, axis=0)
            img_in /= 255.0
            img_in = np.array(img_in)
            batch.append(img_in)
        batch = np.concatenate(batch, 0)
        # batch = torch.tensor(batch)

        detections = self.yolov5_wrapper.run(batch)
        for i, img_path in enumerate(self.image_path_batch):
            parent, filename = os.path.split(img_path)
            save_name = os.path.join('output', filename)
            # Save image
            cv2.imwrite(save_name, plot_boxes(detections[i], batch[i]))
        #print('input->{}, time->{:.2f}ms, saving into output/'.format(self.image_path_batch, use_time * 1000))


class inferThread(threading.Thread):
    def __init__(self, yolov5_wrapper, batch=None):
        threading.Thread.__init__(self)
        self.yolov5_wrapper = yolov5_wrapper
        self.batch = batch

    def run(self):
        return self.yolov5_wrapper.run(self.batch)


if __name__ == "__main__":
    # load custom plugins
    engine_file_path = sys.argv[1]
    image_dir = sys.argv[2]

    categories = ['survivor', 'helmet', 'backpack', 'drill', 'fire_extinguisher', 'rope']

    if os.path.exists('output/'):
        shutil.rmtree('output/')
    os.makedirs('output/')
    # a YoLov5TRT instance
    print(engine_file_path)
    # yolov5_wrapper = OnnxModel(engine_file_path)
    yolov5_wrapper = TRTmodel(engine_file_path)
    # yolov5_wrapper = Engine(engine_file_path)
    try:
        print('batch size is', yolov5_wrapper.batch_size)

        image_path_batches = get_img_path_batches(yolov5_wrapper.batch_size, image_dir)
        t0 = time_synchronized()
        for i, batch in enumerate(image_path_batches):
            if len(batch) != yolov5_wrapper.batch_size:
                continue
            if i > 300:
                break
            # create a new thread to do inference

            thread1 = inferThread(yolov5_wrapper, batch)
            thread1.run()
            #thread1.start()
            #thread1.join()
            print("done 1 batch")
        print('total time: {}'.format(time_synchronized() - t0))
    finally:
        fps = yolov5_wrapper.fps[2:]
        assert len(fps) > 0
        print('average fps: {}'.format(sum(fps) / len(fps)))
        # destroy the instance
        yolov5_wrapper.destroy()
