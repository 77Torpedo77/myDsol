联合优化，需要在关键帧的pose被记录前优化，记录的函数为SetState，函数调用顺序为：
```
Estimate
    └── Map
        └── DirectOdometry.AddKeyframe
                                └── KeyframeWindow.AddKeyframe
                                                    └── SetFrame
                                                            └── SetState
```
故而如果想要联合优化所有帧的pose，在Estimate函数中，在调用Map函数之前联合优化即可。
若想只联合优化关键帧，则需要找到AddKeyframe判断是否添加关键帧的判定bool。

在Track函数之后，status.add_kf = ShouldAddKeyframe()将给出判定bool。 
因此，只优化关键帧，只需在Estimate函数中，在调用Map函数之前，判断status.add_kf的值，若为true，则多加一步联合优化即可。
