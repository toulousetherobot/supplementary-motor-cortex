# Supplementary Motor Cortex

<img src="https://i.imgur.com/51OXfrz.png" alt="Maps used by Horsley and Shafer to summarise the results of cortical stimulation in Ceropithecus monkeys." style="width: 350px;"/>

In biology the supplementary motor cortex or area (SMA) is a part of the brain that contributes to the control of movement. Neurons in the SMA project directly to the spinal cord and may play a role in the direct control of movement. It is hypothesised that it may be the control of movements that are internally generated rather than triggered by sensory events, and the control of sequences of movements. (1)

For **Toulouse the Robot** the SMA similarily provides the internally generated plan to make a sequence of movements for Toulouse the Robot decomposing a series of provided brush strokes as Bezier curves into coordinated motor angles to issue commands out to the primary motor cortex. This algorithm decomposes Bezier curves into a B-Spline curve and De Boor's algorithm to provide a fast and numerically stable way for finding a point on a B-spline curve given a u in the domain and the specific inverse kinematics for Toulouse. 

***
#### References
1. Halsband, U., Matsuzaka, Y. and Tanji, J. (1994). "Neuronal activity in the primate supplementary, pre-supplementary and premotor cortex during externally and internally instructed sequential movements". Neurosci. Res. 20 (2): 149–155. doi:10.1016/0168-0102(94)90032-9. PMID 7808697.
