#Timings

Test Case:
./warping  {warping on/off = 1/0} {mono/stereo = 0/1} {scene: teichplatz/teichplatz+ruine/teichplatz_lod = 0/1/2} {record path: on/off = 1/0}

target fps: 100

##Test PCs
LAB:
    - Titan X (Pascal), 12 GB G5X; 11.4 Gbps; 1582 MHz
    - Intel i7-5930K @ 3.50 GHz (6 cores, 12 logical prozessors)
    - 15.98 GB RAM DDR4(2132) 4 x 4096MB
HOME:
    - GTX 760

## CAM PATH Durations
teichgraben: 3 min == 180 000 ms
teichgraben_ruine: 7 min == 420 000 ms

##Models
Ruine:      11 751 088 faces
Teichplatz: 21 984 286 faces
Wappen:      2 000 000 faces
#### Singlethreaded
Single Thread;
Face count: 35 735 374
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:198] [SINGLE] fps: 2.04321
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0.001024 ms
GPU: BBoxPass - RIGHT : 0.001024 ms
GPU: LightVisibilityPass - LEFT : 175.131 ms
GPU: LightVisibilityPass - RIGHT : 174.902 ms
GPU: LineStripPass - LEFT : 0.001024 ms
GPU: LineStripPass - RIGHT : 0.001024 ms
GPU: ResolvePass - LEFT : 0.4864 ms
GPU: ResolvePass - RIGHT : 0.483328 ms
GPU: TexturedQuadPass - LEFT : 0.001024 ms
GPU: TexturedQuadPass - RIGHT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0 ms
GPU: TriMeshPass - LEFT : 69.0186 ms
GPU: TriMeshPass - RIGHT : 69.0319 ms
Total : 489.06 ms
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:198] [SINGLE] fps: 2.0418
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0 ms
GPU: BBoxPass - RIGHT : 0.001024 ms
GPU: LightVisibilityPass - LEFT : 175.129 ms
GPU: LightVisibilityPass - RIGHT : 174.882 ms
GPU: LineStripPass - LEFT : 0.001024 ms
GPU: LineStripPass - RIGHT : 0.001024 ms
GPU: ResolvePass - LEFT : 0.485376 ms
GPU: ResolvePass - RIGHT : 0.483328 ms
GPU: TexturedQuadPass - LEFT : 0 ms
GPU: TexturedQuadPass - RIGHT : 0 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0.001024 ms
GPU: TriMeshPass - LEFT : 69.0217 ms
GPU: TriMeshPass - RIGHT : 69.0166 ms
Total : 489.023 ms
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:198] [SINGLE] fps: 2.04107
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0.001024 ms
GPU: BBoxPass - RIGHT : 0 ms
GPU: LightVisibilityPass - LEFT : 174.96 ms
GPU: LightVisibilityPass - RIGHT : 174.954 ms
GPU: LineStripPass - LEFT : 0 ms
GPU: LineStripPass - RIGHT : 0.001024 ms
GPU: ResolvePass - LEFT : 0.488448 ms
GPU: ResolvePass - RIGHT : 0.484352 ms
GPU: TexturedQuadPass - LEFT : 0.001024 ms
GPU: TexturedQuadPass - RIGHT : 0 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0 ms
GPU: TriMeshPass - LEFT : 69.0463 ms
GPU: TriMeshPass - RIGHT : 69.3084 ms
Total : 489.246 ms
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:198] [SINGLE] fps: 3.05234
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0.001024 ms
GPU: BBoxPass - RIGHT : 0 ms
GPU: LightVisibilityPass - LEFT : 104.325 ms
GPU: LightVisibilityPass - RIGHT : 103.234 ms
GPU: LineStripPass - LEFT : 0 ms
GPU: LineStripPass - RIGHT : 0.001024 ms
GPU: ResolvePass - LEFT : 0.357376 ms
GPU: ResolvePass - RIGHT : 0.370688 ms
GPU: TexturedQuadPass - LEFT : 0 ms
GPU: TexturedQuadPass - RIGHT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0.001024 ms
GPU: TriMeshPass - LEFT : 70.3939 ms
GPU: TriMeshPass - RIGHT : 69.7139 ms
Total : 348.4 ms

######################################
Single Thread;
Models: Ruine, Teichplatz
Face count: 33 735 374
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:198] [SINGLE] fps: 16.8679
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0.001024 ms
GPU: BBoxPass - RIGHT : 0 ms
GPU: LightVisibilityPass - LEFT : 22.4051 ms
GPU: LightVisibilityPass - RIGHT : 22.0549 ms
GPU: LineStripPass - LEFT : 0.001024 ms
GPU: LineStripPass - RIGHT : 0.001024 ms
GPU: ResolvePass - LEFT : 0.336896 ms
GPU: ResolvePass - RIGHT : 0.3328 ms
GPU: TexturedQuadPass - LEFT : 0.001024 ms
GPU: TexturedQuadPass - RIGHT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0 ms
GPU: TriMeshPass - LEFT : 7.56634 ms
GPU: TriMeshPass - RIGHT : 7.55712 ms
Total : 60.2593 ms
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:198] [SINGLE] fps: 22.6659
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0 ms
GPU: BBoxPass - RIGHT : 0.001024 ms
GPU: LightVisibilityPass - LEFT : 10.0209 ms
GPU: LightVisibilityPass - RIGHT : 10.0219 ms
GPU: LineStripPass - LEFT : 0 ms
GPU: LineStripPass - RIGHT : 0 ms
GPU: ResolvePass - LEFT : 0.36352 ms
GPU: ResolvePass - RIGHT : 0.360448 ms
GPU: TexturedQuadPass - LEFT : 0 ms
GPU: TexturedQuadPass - RIGHT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0 ms
GPU: TriMeshPass - LEFT : 8.05069 ms
GPU: TriMeshPass - RIGHT : 7.70765 ms
Total : 36.5271 ms
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:198] [SINGLE] fps: 26.9929
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0.001024 ms
GPU: BBoxPass - RIGHT : 0.001024 ms
GPU: LightVisibilityPass - LEFT : 10.0516 ms
GPU: LightVisibilityPass - RIGHT : 10.0506 ms
GPU: LineStripPass - LEFT : 0.001024 ms
GPU: LineStripPass - RIGHT : 0.001024 ms
GPU: ResolvePass - LEFT : 0.251904 ms
GPU: ResolvePass - RIGHT : 0.239616 ms
GPU: TexturedQuadPass - LEFT : 0.001024 ms
GPU: TexturedQuadPass - RIGHT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0 ms
GPU: TriMeshPass - LEFT : 7.91654 ms
GPU: TriMeshPass - RIGHT : 7.55098 ms
Total : 36.0673 ms
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:198] [SINGLE] fps: 27.0451
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0.001024 ms
GPU: BBoxPass - RIGHT : 0 ms
GPU: LightVisibilityPass - LEFT : 10.0506 ms
GPU: LightVisibilityPass - RIGHT : 10.0444 ms
GPU: LineStripPass - LEFT : 0 ms
GPU: LineStripPass - RIGHT : 0.001024 ms
GPU: ResolvePass - LEFT : 0.336896 ms
GPU: ResolvePass - RIGHT : 0.325632 ms
GPU: TexturedQuadPass - LEFT : 0.001024 ms
GPU: TexturedQuadPass - RIGHT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0 ms
GPU: TriMeshPass - LEFT : 8.03123 ms
GPU: TriMeshPass - RIGHT : 7.67181 ms
Total : 36.4657 ms
###############################
Single Thread
Models: Ruine
Face count 21 mio
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:198] [SINGLE] fps: 33.3222
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0.001024 ms
GPU: BBoxPass - RIGHT : 0 ms
GPU: LightVisibilityPass - LEFT : 11.1442 ms
GPU: LightVisibilityPass - RIGHT : 10.8104 ms
GPU: LineStripPass - LEFT : 0 ms
GPU: LineStripPass - RIGHT : 0 ms
GPU: ResolvePass - LEFT : 0.236544 ms
GPU: ResolvePass - RIGHT : 0.237568 ms
GPU: TexturedQuadPass - LEFT : 0 ms
GPU: TexturedQuadPass - RIGHT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0.001024 ms
GPU: TriMeshPass - LEFT : 3.62086 ms
GPU: TriMeshPass - RIGHT : 3.62189 ms
Total : 29.6745 ms
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:198] [SINGLE] fps: 34.0929
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0.001024 ms
GPU: BBoxPass - RIGHT : 0 ms
GPU: LightVisibilityPass - LEFT : 9.20883 ms
GPU: LightVisibilityPass - RIGHT : 9.20371 ms
GPU: LineStripPass - LEFT : 0 ms
GPU: LineStripPass - RIGHT : 0 ms
GPU: ResolvePass - LEFT : 0.453632 ms
GPU: ResolvePass - RIGHT : 0.480256 ms
GPU: TexturedQuadPass - LEFT : 0.001024 ms
GPU: TexturedQuadPass - RIGHT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0 ms
GPU: TriMeshPass - LEFT : 4.06733 ms
GPU: TriMeshPass - RIGHT : 3.72531 ms
Total : 27.1421 ms
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:198] [SINGLE] fps: 62.0912
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0 ms
GPU: BBoxPass - RIGHT : 0.001024 ms
GPU: LightVisibilityPass - LEFT : 9.34298 ms
GPU: LightVisibilityPass - RIGHT : 9.33786 ms
GPU: LineStripPass - LEFT : 0 ms
GPU: LineStripPass - RIGHT : 0.001024 ms
GPU: ResolvePass - LEFT : 0.094208 ms
GPU: ResolvePass - RIGHT : 0.094208 ms
GPU: TexturedQuadPass - LEFT : 0.001024 ms
GPU: TexturedQuadPass - RIGHT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0.001024 ms
GPU: TriMeshPass - LEFT : 1.39674 ms
GPU: TriMeshPass - RIGHT : 1.44998 ms
Total : 21.7211 ms
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:198] [SINGLE] fps: 41.6958
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0 ms
GPU: BBoxPass - RIGHT : 0 ms
GPU: LightVisibilityPass - LEFT : 9.16378 ms
GPU: LightVisibilityPass - RIGHT : 8.82995 ms
GPU: LineStripPass - LEFT : 0.001024 ms
GPU: LineStripPass - RIGHT : 0.001024 ms
GPU: ResolvePass - LEFT : 0.28672 ms
GPU: ResolvePass - RIGHT : 0.287744 ms
GPU: TexturedQuadPass - LEFT : 0.001024 ms
GPU: TexturedQuadPass - RIGHT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0 ms
GPU: TriMeshPass - LEFT : 3.67821 ms
GPU: TriMeshPass - RIGHT : 3.67718 ms
Total : 25.9277 ms
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:198] [SINGLE] fps: 32.4569
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0.001024 ms
GPU: BBoxPass - RIGHT : 0 ms
GPU: LightVisibilityPass - LEFT : 10.9332 ms
GPU: LightVisibilityPass - RIGHT : 10.9353 ms
GPU: LineStripPass - LEFT : 0 ms
GPU: LineStripPass - RIGHT : 0 ms
GPU: ResolvePass - LEFT : 0.47616 ms
GPU: ResolvePass - RIGHT : 0.480256 ms
GPU: TexturedQuadPass - LEFT : 0.001024 ms
GPU: TexturedQuadPass - RIGHT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0.001024 ms
GPU: TriMeshPass - LEFT : 4.02944 ms
GPU: TriMeshPass - RIGHT : 3.69459 ms
Total : 30.5541 ms

####Warping
Multi thread
Face count: 35 735 374
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:444] [SLOW] fps: 1.24087
===== Time Queries for Context: 1 ============================
GPU: BBoxPass - LEFT : 0.001024 ms
GPU: BBoxPass - RIGHT : 0.001024 ms
GPU: LightVisibilityPass - LEFT : 299.246 ms
GPU: LightVisibilityPass - RIGHT : 296.859 ms
GPU: LineStripPass - LEFT : 0 ms
GPU: LineStripPass - RIGHT : 0.001024 ms
GPU: ResolvePass - LEFT : 0.461824 ms
GPU: ResolvePass - RIGHT : 0.46592 ms
GPU: SurfaceDetectionPass - LEFT : 0.033792 ms
GPU: SurfaceDetectionPass - RIGHT : 0.033792 ms
GPU: Texture Blitting - LEFT : 5.77024 ms
GPU: Texture Blitting - RIGHT : 0.109568 ms
GPU: TexturedQuadPass - LEFT : 0 ms
GPU: TexturedQuadPass - RIGHT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0.001024 ms
GPU: TriMeshPass - LEFT : 100.709 ms
GPU: TriMeshPass - RIGHT : 99.3833 ms
Total : 803.077 ms
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:661] [FAST] fps: 5.11291
===== Time Queries for Context: 1 ============================
GPU: WarpGridGeneratorPass - LEFT : 0.397312 ms
GPU: WarpGridGeneratorPass - RIGHT : 11.2323 ms
GPU: WarpPass - LEFT : 0.605184 ms
GPU: WarpPass - RIGHT : 0.603136 ms
Total : 12.8379 ms
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:661] [FAST] fps: 5.71944
===== Time Queries for Context: 1 ============================
GPU: WarpGridGeneratorPass - LEFT : 12.8297 ms
GPU: WarpGridGeneratorPass - RIGHT : 0.395264 ms
GPU: WarpPass - LEFT : 0.546816 ms
GPU: WarpPass - RIGHT : 0.55808 ms
Total : 14.3299 ms

[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:444] [SLOW] fps: 1.29074
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0.001024 ms
GPU: BBoxPass - RIGHT : 0.001024 ms
GPU: LightVisibilityPass - LEFT : 292.151 ms
GPU: LightVisibilityPass - RIGHT : 291.063 ms
GPU: LineStripPass - LEFT : 0.001024 ms
GPU: LineStripPass - RIGHT : 0.001024 ms
GPU: ResolvePass - LEFT : 0.49664 ms
GPU: ResolvePass - RIGHT : 0.497664 ms
GPU: SurfaceDetectionPass - LEFT : 0.034816 ms
GPU: SurfaceDetectionPass - RIGHT : 0.033792 ms
GPU: Texture Blitting - LEFT : 5.14458 ms
GPU: Texture Blitting - RIGHT : 0.105472 ms
GPU: TexturedQuadPass - LEFT : 0 ms
GPU: TexturedQuadPass - RIGHT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0.001024 ms
GPU: TriMeshPass - LEFT : 90.4827 ms
GPU: TriMeshPass - RIGHT : 89.4874 ms
Total : 769.504 ms
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:661] [FAST] fps: 5.7559
===== Time Queries for Context: 1 ============================
GPU: WarpGridGeneratorPass - LEFT : 0.396288 ms
GPU: WarpGridGeneratorPass - RIGHT : 30.4312 ms
GPU: WarpPass - LEFT : 0.472064 ms
GPU: WarpPass - RIGHT : 0.475136 ms
Total : 31.7747 ms


######################################
Multi Thread;
Models: Ruine, Teichplatz
Face count: 33 735 374
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:444] [SLOW] fps: 17.0974
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0.001024 ms
GPU: BBoxPass - RIGHT : 0.001024 ms
GPU: LightVisibilityPass - LEFT : 26.1652 ms
GPU: LightVisibilityPass - RIGHT : 9.81094 ms
GPU: LineStripPass - LEFT : 0 ms
GPU: LineStripPass - RIGHT : 0.001024 ms
GPU: ResolvePass - LEFT : 0.376832 ms
GPU: ResolvePass - RIGHT : 0.365568 ms
GPU: SurfaceDetectionPass - LEFT : 0.033792 ms
GPU: SurfaceDetectionPass - RIGHT : 0.031744 ms
GPU: Texture Blitting - LEFT : 0.114688 ms
GPU: Texture Blitting - RIGHT : 0.113664 ms
GPU: TexturedQuadPass - LEFT : 0.001024 ms
GPU: TexturedQuadPass - RIGHT : 0 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0.001024 ms
GPU: TriMeshPass - LEFT : 7.73939 ms
GPU: TriMeshPass - RIGHT : 7.73222 ms
Total : 52.4902 ms
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:444] [SLOW] fps: 18.1225
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0 ms
GPU: BBoxPass - RIGHT : 0.001024 ms
GPU: LightVisibilityPass - LEFT : 9.81811 ms
GPU: LightVisibilityPass - RIGHT : 18.9389 ms
GPU: LineStripPass - LEFT : 0.001024 ms
GPU: LineStripPass - RIGHT : 0.001024 ms
GPU: ResolvePass - LEFT : 0.376832 ms
GPU: ResolvePass - RIGHT : 0.36864 ms
GPU: SurfaceDetectionPass - LEFT : 0.032768 ms
GPU: SurfaceDetectionPass - RIGHT : 0.032768 ms
GPU: Texture Blitting - LEFT : 0.115712 ms
GPU: Texture Blitting - RIGHT : 0.111616 ms
GPU: TexturedQuadPass - LEFT : 0 ms
GPU: TexturedQuadPass - RIGHT : 0 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0 ms
GPU: TriMeshPass - LEFT : 8.08755 ms
GPU: TriMeshPass - RIGHT : 7.7271 ms
Total : 45.6141 ms
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:661] [FAST] fps: 148.589
===== Time Queries for Context: 1 ============================
GPU: WarpGridGeneratorPass - LEFT : 0.4096 ms
GPU: WarpGridGeneratorPass - RIGHT : 0.34304 ms
GPU: WarpPass - LEFT : 0.50688 ms
GPU: WarpPass - RIGHT : 0.512 ms
Total : 1.77152 ms
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:661] [FAST] fps: 194.842
===== Time Queries for Context: 1 ============================
GPU: WarpGridGeneratorPass - LEFT : 0.4096 ms
GPU: WarpGridGeneratorPass - RIGHT : 0.344064 ms
GPU: WarpPass - LEFT : 0.516096 ms
GPU: WarpPass - RIGHT : 0.513024 ms
Total : 1.78278 ms
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:661] [FAST] fps: 118.48
===== Time Queries for Context: 1 ============================
GPU: WarpGridGeneratorPass - LEFT : 0.403456 ms
GPU: WarpGridGeneratorPass - RIGHT : 0.34304 ms
GPU: WarpPass - LEFT : 0.505856 ms
GPU: WarpPass - RIGHT : 0.508928 ms
Total : 1.76128 ms

###############################
Multi Thread
Models: Ruine
Face count 21 mio
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:444] [SLOW] fps: 22.1937
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0.001024 ms
GPU: BBoxPass - RIGHT : 0.001024 ms
GPU: LightVisibilityPass - LEFT : 10.9251 ms
GPU: LightVisibilityPass - RIGHT : 10.9251 ms
GPU: LineStripPass - LEFT : 0.001024 ms
GPU: LineStripPass - RIGHT : 0.001024 ms
GPU: ResolvePass - LEFT : 0.505856 ms
GPU: ResolvePass - RIGHT : 0.504832 ms
GPU: SurfaceDetectionPass - LEFT : 0.032768 ms
GPU: SurfaceDetectionPass - RIGHT : 0.032768 ms
GPU: Texture Blitting - LEFT : 0.105472 ms
GPU: Texture Blitting - RIGHT : 0.106496 ms
GPU: TexturedQuadPass - LEFT : 0.001024 ms
GPU: TexturedQuadPass - RIGHT : 0 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0 ms
GPU: TriMeshPass - LEFT : 3.72224 ms
GPU: TriMeshPass - RIGHT : 3.72019 ms
Total : 30.5869 ms

[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:661] [FAST] fps: 127.067
===== Time Queries for Context: 1 ============================
GPU: WarpGridGeneratorPass - LEFT : 0.410624 ms
GPU: WarpGridGeneratorPass - RIGHT : 0.350208 ms
GPU: WarpPass - LEFT : 0.616448 ms
GPU: WarpPass - RIGHT : 0.603136 ms
Total : 1.98042 ms

[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.13205 ms, fps: 469.033
[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.0265 ms, fps: 493.462
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:661] [FAST] fps: 149.84
===== Time Queries for Context: 1 ============================
GPU: WarpGridGeneratorPass - LEFT : 0.405504 ms
GPU: WarpGridGeneratorPass - RIGHT : 0.340992 ms
GPU: WarpPass - LEFT : 0.612352 ms
GPU: WarpPass - RIGHT : 0.601088 ms
Total : 1.95994 ms

[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.0018 ms, fps: 499.551
[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.00135 ms, fps: 499.664
[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.25605 ms, fps: 443.253
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:661] [FAST] fps: 98.2222
===== Time Queries for Context: 1 ============================
GPU: WarpGridGeneratorPass - LEFT : 0.413696 ms
GPU: WarpGridGeneratorPass - RIGHT : 0.349184 ms
GPU: WarpPass - LEFT : 0.612352 ms
GPU: WarpPass - RIGHT : 0.606208 ms
Total : 1.98144 ms

[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.1523 ms, fps: 464.619
[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.18179 ms, fps: 458.339
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:661] [FAST] fps: 168.189
===== Time Queries for Context: 1 ============================
GPU: WarpGridGeneratorPass - LEFT : 0.412672 ms
GPU: WarpGridGeneratorPass - RIGHT : 0.352256 ms
GPU: WarpPass - LEFT : 0.605184 ms
GPU: WarpPass - RIGHT : 0.605184 ms
Total : 1.9753 ms

[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.179 ms, fps: 458.926
[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.10379 ms, fps: 475.332
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:661] [FAST] fps: 102.965
===== Time Queries for Context: 1 ============================
GPU: WarpGridGeneratorPass - LEFT : 0.40448 ms
GPU: WarpGridGeneratorPass - RIGHT : 0.342016 ms
GPU: WarpPass - LEFT : 0.6144 ms
GPU: WarpPass - RIGHT : 0.6144 ms
Total : 1.9753 ms

[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.12721 ms, fps: 470.1
[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.17896 ms, fps: 458.936
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:661] [FAST] fps: 124.375
===== Time Queries for Context: 1 ============================
GPU: WarpGridGeneratorPass - LEFT : 0.402432 ms
GPU: WarpGridGeneratorPass - RIGHT : 0.344064 ms
GPU: WarpPass - LEFT : 0.612352 ms
GPU: WarpPass - RIGHT : 0.60416 ms
Total : 1.96301 ms

[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.24221 ms, fps: 445.989
[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.12921 ms, fps: 469.658
[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.25329 ms, fps: 443.795
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:661] [FAST] fps: 117.479
===== Time Queries for Context: 1 ============================
GPU: WarpGridGeneratorPass - LEFT : 0.412672 ms
GPU: WarpGridGeneratorPass - RIGHT : 0.34304 ms
GPU: WarpPass - LEFT : 0.611328 ms
GPU: WarpPass - RIGHT : 0.603136 ms
Total : 1.97018 ms

[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:444] [SLOW] fps: 21.6164
===== Time Queries for Context: 0 ============================
GPU: BBoxPass - LEFT : 0.001024 ms
GPU: BBoxPass - RIGHT : 0 ms
GPU: LightVisibilityPass - LEFT : 10.9261 ms
GPU: LightVisibilityPass - RIGHT : 19.1775 ms
GPU: LineStripPass - LEFT : 0 ms
GPU: LineStripPass - RIGHT : 0 ms
GPU: ResolvePass - LEFT : 0.504832 ms
GPU: ResolvePass - RIGHT : 0.505856 ms
GPU: SurfaceDetectionPass - LEFT : 0.033792 ms
GPU: SurfaceDetectionPass - RIGHT : 0.033792 ms
GPU: Texture Blitting - LEFT : 0.109568 ms
GPU: Texture Blitting - RIGHT : 0.109568 ms
GPU: TexturedQuadPass - LEFT : 0.001024 ms
GPU: TexturedQuadPass - RIGHT : 0 ms
GPU: TexturedScreenSpaceQuadPass - LEFT : 0.001024 ms
GPU: TexturedScreenSpaceQuadPass - RIGHT : 0 ms
GPU: TriMeshPass - LEFT : 3.72019 ms
GPU: TriMeshPass - RIGHT : 3.71712 ms
Total : 38.8413 ms

[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.23221 ms, fps: 447.987
[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.22864 ms, fps: 448.704
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:661] [FAST] fps: 129.73
===== Time Queries for Context: 1 ============================
GPU: WarpGridGeneratorPass - LEFT : 0.412672 ms
GPU: WarpGridGeneratorPass - RIGHT : 0.350208 ms
GPU: WarpPass - LEFT : 0.612352 ms
GPU: WarpPass - RIGHT : 0.600064 ms
Total : 1.9753 ms

[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.2289 ms, fps: 448.651
[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.23 ms, fps: 448.43
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:661] [FAST] fps: 121.651
===== Time Queries for Context: 1 ============================
GPU: WarpGridGeneratorPass - LEFT : 0.402432 ms
GPU: WarpGridGeneratorPass - RIGHT : 0.34304 ms
GPU: WarpPass - LEFT : 0.615424 ms
GPU: WarpPass - RIGHT : 0.601088 ms
Total : 1.96198 ms

[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.13 ms, fps: 469.484
[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.17664 ms, fps: 459.423
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:661] [FAST] fps: 145.265
===== Time Queries for Context: 1 ============================
GPU: WarpGridGeneratorPass - LEFT : 0.4096 ms
GPU: WarpGridGeneratorPass - RIGHT : 0.351232 ms
GPU: WarpPass - LEFT : 0.620544 ms
GPU: WarpPass - RIGHT : 0.601088 ms
Total : 1.98246 ms

[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.20441 ms, fps: 453.637
[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.1282 ms, fps: 469.882
[GUA][W][C:\Users\Jojo\Documents\git\guacamole\examples\warping\main.cpp:426] [APP] Frame time: 2.1313 ms, fps: 469.198
[GUA][M][C:\Users\Jojo\Documents\git\guacamole\src\gua\renderer\Renderer.cpp:661] [FAST] fps: 115.586
===== Time Queries for Context: 1 ============================
GPU: WarpGridGeneratorPass - LEFT : 0.406528 ms
GPU: WarpGridGeneratorPass - RIGHT : 0.342016 ms
GPU: WarpPass - LEFT : 0.611328 ms
GPU: WarpPass - RIGHT : 0.608256 ms
Total : 1.96813 ms