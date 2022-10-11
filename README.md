# orne_voicevox_core


このパッケージは、[voicevox_core(0.13.0)](https://github.com/VOICEVOX/voicevox_core/tree/0.13.0)をROSで動かすためのパッケージです。
##  Quick Start
### インストール

```bash
git clone https://github.com/Tsumoridesu/orne_voicevox_core.git
```
ビルド
```bash
cd ../
catkin build orne_voicevox_core
source ~/catkin_ws/devel/setup.bash 
```


### 環境構築
[本家のconfigure.py](https://github.com/VOICEVOX/voicevox_core/blob/0.13.0/configure.py)を改造して、使用します。
環境構築するときは、[本家のReleases(VOICEVOX CORE 0.13.1)](https://github.com/VOICEVOX/voicevox_core/releases/tag/0.13.1)から、ビルド済みのコアライブラリをダウンロードし、使用します。
```bash
python configure.py --use_cuda
pip install -r requirements.txt
pip install .
```

### 起動
```bash
roslaunch orne_voicevox_core test.launch
```
提示音を鳴らすまでに少々待ちます。

新しい端末を起動して
```bash
rosservice call /speak これは本当に実行できているんですか 1
```
サービスの引数

```text```：読み上げるテキスト

```speaker_id```：話者のID

### パラメータ調整
```service_name```:
```string```サービスの名前を変更できます

```cpu_num_threads```:
```int```CPUでのスレッド数を変更できます

```use_gpu```:
```bool```gpuを使うかどうか、TrueときCUDA環境とCUDNNをインストールする必要

```openjtalk_dic_path```:
```string```openjtalk辞書のpathです

## ライセンス
[本家のLICENSE](https://github.com/VOICEVOX/voicevox_core)を継承します

使用する[コアライブラリ](https://github.com/VOICEVOX/voicevox_core/releases)が本家と同じく別のライセンスなので注意してください