import torch_pruning as tp
import torch
import argparse
import torch.nn as nn


parser = argparse.ArgumentParser()
parser.add_argument('--weights', type=str, default='yolov5m6.pt', help='initial weights path')
parser.add_argument('--save-name', type=str, default='pruned.pt', help='save name')
parser.add_argument('--prune-amount', type=float, default=0.1, help='Label smoothing epsilon')

opt = parser.parse_args()

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

ckpt = torch.load(opt.weights, map_location=device)  # load checkpoint
model = Model(ckpt['model'].yaml, ch=3, nc=6)  # create
exclude = []  # exclude keys
state_dict = ckpt['model'].float().state_dict()  # to FP32
state_dict = intersect_dicts(state_dict, model.state_dict(), exclude=exclude)  # intersect
model.load_state_dict(state_dict, strict=False)  # load

model.eval()
num_params_before_pruning = tp.utils.count_params(model)
# 1. build dependency graph
strategy = tp.strategy.L1Strategy()
DG = tp.DependencyGraph()
out = model(torch.randn([1,3, 1280, 1280]))
DG.build_dependency(model, example_inputs=torch.randn([1,3, opt.img_size[0], opt.img_size[0]]))
excluded_layers = list(model.model[-1].modules())
for m in model.modules():
    if isinstance(m, nn.Conv2d) and m not in excluded_layers:
        pruning_plan = DG.get_pruning_plan( m, tp.prune_conv, idxs=strategy(m.weight, amount=opt.prune_amount))
        # execute the plan (prune the model)
        pruning_plan.exec()
num_params_after_pruning = tp.utils.count_params( model )
print( "Params reduced from %s to %s"%( num_params_before_pruning, num_params_after_pruning))

#################################################
# Torch Pruning (End)
#################################################
model = model.to(device)


ckpt = {'epoch': 0,
'model': deepcopy(de_parallel(model)).half(),
'wandb_id': wandb_logger.wandb_run.id if wandb_logger.wandb else None,
'optimizer':None}

            
torch.save(ckpt, opt.save_name)