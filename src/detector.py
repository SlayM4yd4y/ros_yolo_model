import cv2
import torch
from ultralytics import YOLO
import argparse
import os

class YOLOModelDetector:
    def __init__(self, model_path, input_path, save_output, output_path=None):
        self.model_path = model_path
        self.input_path = input_path
        self.save_output = save_output
        self.output_path = output_path
        self.model = YOLO(model_path)

    def detect(self):
        # Kép vagy videó betöltése
        cap = cv2.VideoCapture(self.input_path)
        if not cap.isOpened():
            print(f"Error opening file: {self.input_path}")
            return
        
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            
            # Objektum detektálás
            results = self.model(frame)
            
            # Eredmények megjelenítése
            annotated_frame = results.plot()
            cv2.imshow('YOLO Detection', annotated_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
        
        # Eredmény mentése, ha szükséges
        if self.save_output and self.output_path:
            cv2.imwrite(self.output_path, annotated_frame)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='YOLOv8 model detector')
    parser.add_argument('--model_path', type=str, required=True, help='Path to the YOLO model')
    parser.add_argument('--input_path', type=str, required=True, help='Path to the input image/video')
    parser.add_argument('--save_output', type=bool, required=False, default=False, help='Save detection result')
    parser.add_argument('--output_path', type=str, required=False, help='Path to save the detection result')

    args = parser.parse_args()
    
    detector = YOLOModelDetector(args.model_path, args.input_path, args.save_output, args.output_path)
    detector.detect()
