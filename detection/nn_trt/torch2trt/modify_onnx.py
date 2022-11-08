import onnx

path = 'yolov5m6.onnx'

onnx_model = onnx.load(path)
output = [node.name for node in onnx_model.graph.output]
rm = []
for node in onnx_model.graph.output:
    if node.name == 'output':
        rm.append(node)
for node in rm:
    onnx_model.graph.output.remove(node)

endpoint_names = ['output']
for i in range(len(onnx_model.graph.node)):
    for j in range(len(onnx_model.graph.node[i].output)):
        if onnx_model.graph.node[i].output[j] in endpoint_names:
            print('-'*60)
            print(onnx_model.graph.node[i].name)
            print(onnx_model.graph.node[i].input)
            print(onnx_model.graph.node[i].output)


for input in onnx_model.graph.input:
    print (input.name, end=": ")
    # get type of input tensor
    tensor_type = input.type.tensor_type
    # check if it has a shape:
    if (tensor_type.HasField("shape")):
        # iterate through dimensions of the shape:
        for d in tensor_type.shape.dim:
            # the dimension may have a definite (integer) value or a symbolic identifier or neither:
            if (d.HasField("dim_value")):
                print (d.dim_value, end=", ")  # known dimension
            elif (d.HasField("dim_param")):
                print (d.dim_param, end=", ")  # unknown dimension with symbolic name
            else:
                print ("?", end=", ")  # unknown dimension with no name
    else:
        print ("unknown rank", end="")
    print()

print([node.name for node in onnx_model.graph.output])

onnx.checker.check_model(onnx_model)  # check onnx model
onnx.save(onnx_model,path)