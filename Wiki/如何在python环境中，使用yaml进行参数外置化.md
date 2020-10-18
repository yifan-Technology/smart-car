# 如何在python环境中，使用yaml进行参数外置化



## yaml写作方式
```yaml
一级目录1：
	二级目录1：对应数据1
	二级目录2: 对应数据2
一级目录2：
	二级目录3：对应数据3
```

## python环境下调用
```python
import yaml

with open("config.yaml", "r") as f:
    config = yaml.load(f)

# 然后我可以用
print(config['一级目录1']['二级目录1'])

```

```shell
对应数据1
```