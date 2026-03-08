from pathlib import Path

from ultralytics import YOLO


def convert_model(model_path: Path, height: int, width: int) -> None:
    model = YOLO(model_path)
    model.export(
        format="onnx",
        imgsz=(height, width),
        simplify=True,
        name=model_path.stem + "-" + str(width) + "x" + str(height) + ".onnx",
    )


if __name__ == "__main__":
    height = 1088 / 2
    width = 1280 / 2

    convert_model(Path("./yolo26m-finetune.pt"), height, width)
    convert_model(Path("./yolo26s-finetune.pt"), height, width)
    convert_model(Path("./yolo26n-finetune.pt"), height, width)
