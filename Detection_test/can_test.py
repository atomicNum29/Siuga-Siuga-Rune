import ultralytics
from ultralytics import YOLO

def main():
    model = YOLO('./runs/detect/train2/weights/best.pt').to('cuda')
    results = model(source='*.jpg', save=True)

if __name__ == '__main__':
    main()