import torch
from ultralytics import YOLO
import argparse
import os

class YOLOModelTrainer:
    def __init__(self, model_size, train_data, val_data, test_data, output_dir):
        self.model_size = model_size
        self.train_data = train_data
        self.val_data = val_data
        self.test_data = test_data
        self.output_dir = output_dir
        self.model = self.load_model()

    def load_model(self):
        # YOLOv8 model betöltése a megfelelő méretben (n, s, m, l, x)
        model = YOLO(f'yolov8{self.model_size}.pt')
        return model

    def train_model(self):
        # Modell betanítása
        results = self.model.train(data=self.train_data, epochs=100, imgsz=640, val=self.val_data, device=0)
        self.model.save(os.path.join(self.output_dir, 'sajat_model.pt'))

    def validate_model(self):
        # Validációs futtatás
        results = self.model.val(data=self.val_data)
        return results

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='YOLOv8 model trainer')
    parser.add_argument('--model_size', type=str, required=True, help='Model size: n, s, m, l, x')
    parser.add_argument('--train_data', type=str, required=True, help='Training data folder')
    parser.add_argument('--val_data', type=str, required=True, help='Validation data folder')
    parser.add_argument('--test_data', type=str, required=True, help='Testing data folder')
    parser.add_argument('--output_dir', type=str, required=True, help='Output directory for the model')
    
    args = parser.parse_args()
    
    trainer = YOLOModelTrainer(args.model_size, args.train_data, args.val_data, args.test_data, args.output_dir)
    trainer.train_model()
    results = trainer.validate_model()
    print("Model validation complete. Results: ", results)
