import yaml

with open("config.yaml","r") as f:
    config = yaml.load(f, Loader=yaml.FullLoader)

print(config["test1"]["minVolt"])

