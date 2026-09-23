# これはアノマロカリスのシミュレーションを行う

## venvの導入方法
anomalocarsi_sim内の直下に下記コマンドを打って確認
## venvの起動方法
window → .\venv\Scripts\Activate.ps1
## 実行方法
Linux & Windows → python ○○.py
で可能
## venv内に入れるライブラリ
pip install mujoco
## 入っているライブラリは下記(2026/9/22時点)
##(.venv) sanrobo@sanrobo:~/anomalocaris_sim$ pip list
Package           Version \
----------------- --------
absl-py           2.5.0 \
etils             1.14.0 \
fsspec            2026.7.0 \
glfw              2.10.2 \
mujoco            3.12.0 \
numpy             2.5.2 \
pip               24.0  \
PyOpenGL          3.1.10 \
typing_extensions 4.16.0 \
zipp              4.1.0 \
## コードの説明
### water_sim.py
基本的にここでシミュレーションを行う\
このコードは下記のことが可能\
・水中シミュレーション\
・結果を.csvファイルに変換\

### competion.py
コードの比較ファイルを作成するもの
