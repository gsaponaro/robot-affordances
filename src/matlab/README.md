Hand Tool Affordances
=====================

## Dependencies

- MATLAB
- [pmtk3](https://github.com/probml/pmtk3) (run `initPmtk3.m`)

## Indexes of hand postures, objects and tools

Hand postures:

```
straight 1
fortyfive 2
bent 3
```

Objects:

```
pear 4
lemon 5
wball 6
ylego 7
```

Tools:

```
hook 8
stick 9
rake 10
```

## Indexes of motor actions

```
tapFromRight 1
tapFromLeft 2
draw 3
push 4
```

## Tool selection scripts

1. `train_hand_select_manipulator.m` expects a query with H/T features + O features + A index, similarly to the POETICON++ MATLAB-yarp script `predictor.m` (however now we have a bigger feature vector, and the action indexes have changed). For now it operates on the `pca_2n2c2v_ht` network (number of components c=2, training 100% hand data). The output is the 5x5 effect prediction matrix along the 5 bins.

Example:
```
>> t = [0.770997 0.226286 0.293183 0.208899 0.704147 0.450975 -0.019543 0.024588 0.172989 -0.001942 -0.001238 0.001102];
>> o = [0.854576 0.393378 0.525345 0.343063 0.677111 0.057134 -0.059711 0.186663 0.000304 -0.004093 0.008576 -0.001682];
>> a = 3; % draw
>> query = [t o a];
>> posterior = train_hand_select_manipulator(query)
posterior =
         0    0.0002    0.0051    0.0011    0.0003
         0    0.0007    0.0190    0.0041    0.0012
         0    0.0125    0.3167    0.0681    0.0194
         0    0.0095    0.2407    0.0517    0.0148
         0    0.0070    0.1786    0.0384    0.0110
```

2. `select_manipulator_evaluation_*t.m` loads `tool_cont_desc_noheader.txt` (tools and objects descriptors) and cycles over the ones with IDs = the tools and objects that we want to test, given an action index. It computes a fractional **evaluation criterion of the posterior** for each tool, for example
```
toolQualityTapFromRight = select_tool_evaluation(1) % action 1 is tapFromRight
toolQualityTapFromRight =
    1.0000    0.5000    0.3333   % hook, stick, rake
```

3. the **evaluation criterion of the posterior** is defined in `posterior_evaluation_criterion.m` (there are multiple possible criteria).
