# ForRocket(C++) Coding Rule

---

## 総則
基本的にはGoogle C++ Coding Style Guideに準じる。

## 命名規則

### クラス/構造体
PascalCase

### 関数
PascalCase or camelCase

### 変数
snake_case

## インデント
space 4

## ブロックインデント開始位置

``` C++
void Function() {
    for (int i = 0; i < 100; ++i) {
        if (i == 1) {
            break;
        }
    }
}
```