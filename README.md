# m5stack_MPU9250

Madgwick FilterとMPU9250 digital motion processing (DMP)を利用した姿勢推定デモです。  
シリアル経由で姿勢データを送信、Processingで可視化します。  

## Arduino依存ライブラリ

### SparkFun MPU-9250 Digital Motion Processor (DMP) Arduino Library
https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library

ライブラリインストールフォルダ以下 `SparkFun_MPU-9250-DMP_Arduino_Library-master/src/util/inv_mpu.c` を修正します。

```
#include <Arduino.h>
+#ifndef min
+#define min _min
+#endif
#define MPU9250
```

### Madgwick Library
https://github.com/arduino-libraries/MadgwickAHRS

## Visual Studio CodeでPROCESSING実行メモ
拡張機能をインストール  
https://marketplace.visualstudio.com/items?itemName=Tobiah.language-pde

タスクを作成しtasks.jsonの `args` を修正します。  
サブフォルダ以下（ここではviewer/viewer.pde）で、ビルドタスクの実行（Ctrl + Shift + B）により起動が可能。  

```
"args": [
        "--force",
        "--sketch=${fileDirname}",
        "--output=${workspaceRoot}/out",
        "--run"
      ],
```
