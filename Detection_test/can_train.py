# import yaml

# data = { 'train' : '../train/images/',
#          'val' : '../val/images/',
#          'test' : '../test/images/',
#          'names' : ['can'],
#          'nc' : 1 }

# with open('./can_data/data.yaml', 'w') as f:
#     yaml.dump(data, f)

import ultralytics
from ultralytics import YOLO

def main():
    model = YOLO('yolo11n.pt').to('cuda')
    model.train(data='C:/Users/yb/Siuga-Siuga-Rune/Detection_test/can_data/data.yaml', epochs=30, patience=0, imgsz=640, batch=8)

if __name__ == '__main__':
    main()