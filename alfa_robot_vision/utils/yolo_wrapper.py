import cv2
import numpy as np
try:
    import torch
except ImportError:
    torch = None
try:
    from ultralytics import YOLO
except ImportError:
    print("Warning: 'ultralytics' not found. Please install it via 'pip install ultralytics'")
    YOLO = None

class YoloWrapper:
    def __init__(self, model_path, conf_thresh=0.5):
        """
        Initialize YOLO detector with a custom model.
        Args:
            model_path (str): Path to the .pt model file.
            conf_thresh (float): Confidence threshold for detections.
        """
        self.conf_thresh = conf_thresh
        self.model = None
        self.device = 'cpu'

        if YOLO is None:
            return

        print(f"[YoloWrapper] Loading model from: {model_path}")
        try:
            self.model = YOLO(model_path)
            # Check device
            self.device = 'cuda' if (torch and torch.cuda.is_available()) else 'cpu'
            print(f"[YoloWrapper] Model loaded successfully on {self.device}")
        except Exception as e:
            print(f"[YoloWrapper] Error loading model: {e}")
            self.model = None

    def detect(self, cv_image, classes=None):
        """
        Run inference on an image.
        Args:
            cv_image (numpy.ndarray): Input image (BGR).
            classes (list): Optional list of class names/IDs to filter.
            
        Returns:
            list: List of dicts with keys ['class', 'class_id', 'conf', 'bbox', 'mask_poly']
        """
        if self.model is None:
            return []

        # Run inference
        # verbose=False to keep logs clean
        try:
            results = self.model(cv_image, conf=self.conf_thresh, verbose=False)
        except Exception as e:
            print(f"[YoloWrapper] Inference error: {e}")
            return []
        
        if not results:
            return []

        result = results[0] # We only processed one image
        detections = []

        # Process Boxes
        if result.boxes is not None:
            boxes = result.boxes.xyxy.cpu().numpy() # x1, y1, x2, y2
            confs = result.boxes.conf.cpu().numpy()
            cls_ids = result.boxes.cls.cpu().numpy().astype(int)
            
            masks_xy = []
            if result.masks is not None:
                masks_xy = result.masks.xy

            for i in range(len(boxes)):
                class_id = cls_ids[i]
                class_name = result.names[class_id]
                
                # Filter by class if requested
                if classes and class_name not in classes:
                    continue

                det = {
                    "class": class_name,
                    "class_id": int(class_id),
                    "conf": float(confs[i]),
                    "bbox": boxes[i].tolist(), # [x1, y1, x2, y2]
                    "mask_poly": None
                }

                # Attach mask if available (Segment Model)
                if i < len(masks_xy):
                    det["mask_poly"] = masks_xy[i]
                
                detections.append(det)

        return detections
