import cv2
import numpy as np
import torch

from light_inference import light_run, load_light_weights
from models.experimental import attempt_load
from utils.general import non_max_suppression

class SequoiaDetector(object):
    def __init__(self, weights_dir, conf_threshold=0.4, use_light=False):
        if torch.cuda.is_available():
            self.device = torch.device("cuda:0")
        else:
            self.device = torch.device("cpu")

        self.weights_dir = weights_dir or ""
        self.weights = 'sequoiaV1.pt'
        self.model = attempt_load(self.weights_dir + "/" + self.weights, map_location=self.device)

        self.light_weights = 'light_classifier_v1.th'
        self.use_light = use_light
        if self.use_light:
            load_light_weights(self.light_weights)

        self.conf_threshold = conf_threshold

        self.orig_img = None
        self.width = 0
        self.height = 0
        self.work_img = None
        self.predictions = None
        self.labels = ['t', 'ct']
        self.bboxes = {l:[] for l in self.labels}

    def _prep_img(self):
        # convert img to correct size and format
        img = cv2.resize(self.orig_img,(512, 512)).astype(np.float32)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        img = np.swapaxes(img, 0, 2)
        img = np.swapaxes(img, 1, 2)
        img = img.reshape(1, 3, 512, 512)
        self.work_img = torch.from_numpy(img).to(self.device) / 255.0 # cast to gpu and normalize

    def _yolo_detect(self):
        pred = self.model(self.work_img)[0]
        self.predictions = non_max_suppression(pred)[0]

    def _light_classify(self, bbox):
        # light classifier seems to have reversed mapping (0:t, 1:ct)
        return 1 - light_run(self.orig_img, bbox).item()

    def visualize(self):
        viz_img = self.orig_img.copy()
        for label, bboxes in self.bboxes.items():
            for x1, y1, x2, y2 in bboxes:
                viz_img = cv2.rectangle(viz_img, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
                viz_img = cv2.putText(viz_img, label, (int(x1),int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)

        return viz_img

    def detect(self, img: np.ndarray):
        self.orig_img = img
        self.height, self.width, _ = img.shape
        self.predictions = None
        self.bboxes = {l:[] for l in self.labels}
        self._prep_img()

        # Run inference
        self._yolo_detect()

        if self.predictions is not None and len(self.predictions):            
            for det in self.predictions:
                x1, y1, x2, y2, conf, pred = det.tolist()

                # scale coords
                x1 *= self.width/512
                x2 *= self.width/512
                y1 *= self.height/512
                y2 *= self.height/512

                bbox = (x1,y1,x2,y2)
                if conf >= self.conf_threshold:
                    if self.use_light:
                        pred = self._light_classify(bbox)
                    
                    pred_type = 't' if pred > 0.5 else 'ct'
                    self.bboxes[pred_type].append(bbox)

        return self.bboxes


if __name__ == '__main__':
    img = cv2.imread('/app/images/test/de_dust20135.jpg')
    detector = SequoiaDetector()
    bboxes = detector.detect(img)
