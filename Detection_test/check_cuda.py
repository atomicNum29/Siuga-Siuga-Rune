import torch

# CUDA 사용 가능 여부 확인
if torch.cuda.is_available():
    print("CUDA is available! Using GPU.")
else:
    print("CUDA is not available. Using CPU.")
