# Active AI model configuration, published by mind nodes for introspection.

string model_name
string precision        # "fp32", "fp16", "int8"
string backend          # "tensorrt", "dla", "cpu"
string task             # "depth", "detection", "segmentation"
float32 input_width
float32 input_height
float32 fps             # measured inference rate
