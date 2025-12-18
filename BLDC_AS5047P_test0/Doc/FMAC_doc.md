# FMACの実装検討

- BLDCMD用に速度制御を行うプログラムを書きたい
- しかし、エンコーダの値をそのまま使って角速度を出すと誤差が大きいため、FMACを用いたフィルタで角速度推定をしたい

## MDの仕様
- エンコーダー
    - アブソリュートエンコーダ(AS5047P)
    - 解像度:14bit(0~16383)
- 制御
    - 制御周期:20kHz

## 経緯
- 注：ここからは、説明を簡単にするために実際の変数名を用いて書く。以下に各変数の説明を示す
    ```
    //angle
    uint16_t theta_mech_curr = 0; //現在のロータ角度。エンコーダの値(0~16383)がそのまま入る
    uint16_t theta_mech_prev = 0; //1ループ前のロータ角度
    uint32_t theta_mech_curr_x256 = 0; //現在のロータ角度を、後の計算での精度を上げるために256倍したもの
    uint32_t theta_mech_prev_x256 = 0; //1ループ前のロータ角度を、後の計算での精度を上げるために256倍したもの

    //speed
    int16_t omega_mech_curr = 0; //現在のロータ角速度
    int16_t omega_mech_curr_x256 = 0; //現在のロータ角速度を、後の計算での精度を上げるために256倍したもの
    ```
- 速度制御を行うためには現在のロータ角速度を得る必要がある。当初は`omega_mech_curr = ((int16_t)(theta_mech_curr - theta_mech_prev))`として角速度を求めていたが、思ったよりもエンコーダの誤差や量子化誤差の影響が大きく、角速度の値に大きくノイズが乗ってしまい、速度制御がうまくいかなかった(ピーーっていう高い音が鳴ってうるさい)。
    - 例として、角速度の真値が`omega_mech_curr = 1`の場合を考えてみる。これはつまり、1ループで角度が1増える速度でロータが回転している場合である。
    - しかし、エンコーダから得られる角度データには誤差があり、大抵1か2くらい値がブレる。したがって、実際に得られる角速度は誤差によって-1~3くらいの値をとることが多い。こうなると、真値とはかけ離れた値になってしまい、速度制御もまともにできなくなってしまう。
    - さらに、制御周期は20kHzなので、`omega_mech_curr = 1`のときロータのRPSは1.221くらいになり、結構早く回っていることになる。本来もっと低速で微調整できるようにしたいが、RPS=1.221で誤差の影響を強く受けているようでは確実に使い物にならない。
- 誤差の影響は速度制御の周期を落とせばある程度低減される。しかし、なんとなくもったいない気がするので、制御周期は落としたくない。そこで、角速度をフィルタリングすることで誤差を小さくできるのではないかと考えた。

## 実装したいフィルタ
- 最小二乗法を使ったフィルタを実装したい。
    - 窓幅Nで、N個の角速度データを入力とし、現在の角速度の推定値を求める。
- 考え方としては、横軸時間、縦軸角度として角度データに最小二乗法で近似直線を引くと、その傾きが角速度になるというのを用いる。
    - ただし、角度のアンラップとの兼ね合いから、入力データは角度ではなく角速度にする。
    - 今回の場合、角度の取得周期が一定であることを利用すれば角速度データに対する重みづけ係数を事前に求めることができ、この係数と角速度データとの畳み込みで推定値を求めることができる。
    - 窓幅を`FMAC_width`、重みづけ係数を`FMAC_coeff[FMAC_width]`とすれば、`FMAC_coeff[i]`は以下のように求まる。
    ```
    for(int i=0; i<FMAC_width; i++){
        FMAC_coeff[i] = ((6 * (i+1) * (FMAC_width - i)))
                        /(FMAC_width * FMAC_width * FMAC_width + 3 * FMAC_width * FMAC_width + 2 * FMAC_width);
    }
    ```
    また、入力データを`FMAC_in[i]`とすれば、出力される角速度の推定値`omega_mech`は以下のような計算で求まる。(`FMAC_in[i]`について、iが小さいほど古いデータ、大きいほど新しいデータである)
    ```
    for(uint8_t i=0; i<width_FMAC; i++){
        omega_mech += FMAC_coeff[i] * FMAC_in[i];
    }
    ```
- これをFMACを用いて実装したい。

## FMACでの実装の要件
- FIRで計算
- 窓幅20

## おためしコード
- FMACのセッティングを行うコードを書く
- セッティングの要件を以下に箇条書きする
    - CLIP : Enable, 全てのDMA&割り込み : Disable
    - X1BUF : FULL_WM=0, BUF_SIZE=20, BASE_ADDRESS=20
    - X2BUF : BUF_SIZE=20, BASE_ADDRESS=0
    - YBUF : FULL_WM=0, BUF_SIZE=1, BASE_ADDRESS=40
- バッファにプリロードするのは、
    - X1BUF : 全て0
    - X2BUF : FMAC_coeff[i]
    - YBUF : 0

## 変更・検討してほしい点
- 関数化する必要も一旦ないです。
- includeやdefineなどのコードも、すでに書いてある前提として削除してかまいません(下記単体でコードとして成り立ってなくても構わない)
- FMAC_coeff[i]は、はじめからFMACに適したスケーリングで固定小数点で持っていると考えて構いません。したがってプリロードの際に変換・スケーリングする必要もありません。
- FMACにおけるFIRフィルタでの畳み込みの仕様的に、FMAC_coeff[i]の値をX2バッファに順番に入れていくと、逆向きで畳み込みされる(実際は`for(uint8_t i=0; i<width_FMAC; i++){omega_mech += FMAC_coeff[width_FMAC-1-i] * FMAC_in[i];}`みたいな感じで計算されてしまう)可能性はありませんか。

以下は、要件どおりにFMAC関連レジスタへ値を書き込む“べた書き”のCコード例です。
（使用するデバイスヘッダやビット定義に合わせてマクロ名やビット位置を調整してください）



```c

//表記ルール:レジスタへの代入
//reg = FMAC->XXXX;       // レジスタ読み出し
//reg &= ~((0xYY << n) | (0xZZ << m));   // ビットマスク
//reg |= ((0xYY << n) | (0xZZ << m));    // ビット設定
//FMAC->XXXX = reg;       // レジスタ書き込み

#define FMAC_window 20

void FMAC_Setup_and_Preload(void)
{
    extern int32_t FMAC_coeff[FMAC_window];
    uint32_t reg;

    /* reset FMAC */
    reg = FMAC->CR;
    reg &= ~(0x1 << 16);// mask RESET bit
    reg |= (0x1 << 16);// RESET=1
    FMAC->CR = reg;

    HAL_Delay(1);// wait for reset

    reg = FMAC->CR;
    reg &= ~(0x1 << 16);// mask RESET bit
    reg |= (0x0 << 16);// RESET=0
    FMAC->CR = reg;


    /* set FMAC config */
    //reset START bit
    reg = FMAC->PARAM;
    reg &= ~(0x1 << 31);// mask START bit
    reg |= (0x0 << 31);// START=0
    FMAC->PARAM = reg;

    //set CR register
    reg = FMAC->CR;
    reg &= ~(0x0000831F);// mask CLIPEN, DMA, IRQ bits
    reg |= (0x1 << 15);// CLIPEN=1
    FMAC->CR = reg;

    //set X1BUFCFG register
    reg = FMAC->X1BUFCFG;
    reg &= ~((0x3 << 24) | (0xFF << 8) | 0xFF);// mask FULL_WM, BUF_SIZE, BASE bits
    reg |= ((0x0 << 24) | (FMAC_window << 8) | FMAC_window);// FULL_WM=0, SIZE=20, BASE=20
    FMAC->X1BUFCFG = reg;

    //set X2BUFCFG register
    reg = FMAC->X2BUFCFG;
    reg &= ~((0xFF << 8) | 0xFF);//mask BUF_SIZE, BASE bits
    reg |= ((FMAC_window << 8) | 0x0);// SIZE=20, BASE=0
    FMAC->X2BUFCFG = reg;

    //set YBUFCFG register
    reg = FMAC->YBUFCFG;
    reg &= ~((0x3 << 24) | (0xFF << 8) | 0xFF);// mask FULL_WM, BUF_SIZE, BASE bits
    reg |= ((0x0 << 24) | (0x1 << 8) | FMAC_window*2);// FULL_WM=0, SIZE=1, BASE=40
    FMAC->YBUFCFG = reg;


    /* start preload */
    //preload X1 with 0
    reg = FMAC->PARAM;
    reg &= ~(0x1 << 31);// mask START bit
    reg |= (0x0 << 31);// START=0
    FMAC->PARAM = reg;

    reg = FMAC->PARAM;
    reg &= ~((0x7F << 24) | 0xFF);// mask FUNC, P bits
    reg |= ((0x1 << 24) | FMAC_window);// FUNC=1(X1), P=20
    FMAC->PARAM = reg;

    reg = FMAC->PARAM;
    reg &= ~(0x1 << 31);// mask START bit
    reg |= (0x1 << 31);// START=1
    FMAC->PARAM = reg;

    for (int i=0; i<FMAC_window; i++) {
        WRITE_REG(FMAC->WDATA, 0);
    }
    while ((READ_REG(FMAC->PARAM) & (0x1 << 31)) != 0) {}

    // preload X2 with FMAC_coeff[]
    reg = FMAC->PARAM;
    reg &= ~(0x1 << 31);// mask START bit
    reg |= (0x0 << 31);// START=0
    FMAC->PARAM = reg;

    reg = FMAC->PARAM;
    reg &= ~((0x7F << 24) | (0xFF << 8) | 0xFF);// mask FUNC, Q, P bits
    reg |= ((0x2 << 24) | (0x0 << 8) | FMAC_window);// FUNC=2(X2), Q=0, P=20
    FMAC->PARAM = reg;

    reg = FMAC->PARAM;
    reg &= ~(0x1 << 31);// mask START bit
    reg |= (0x1 << 31);// START=1
    FMAC->PARAM = reg;

    for (int i=0; i<FMAC_window; i++) {
        WRITE_REG(FMAC->WDATA, FMAC_coeff[i]);
    }
    while ((READ_REG(FMAC->PARAM) & (0x1 << 31)) != 0) {}

    // preload Y with 0
    reg = FMAC->PARAM;
    reg &= ~(0x1 << 31);// mask START bit
    reg |= (0x0 << 31);// START=0
    FMAC->PARAM = reg;

    reg = FMAC->PARAM;
    reg &= ~((0x7F << 24) | 0xFF);// mask FUNC, P bits
    reg |= ((0x3 << 24) | 0x1);// FUNC=3(Y), P=1
    FMAC->PARAM = reg;

    reg = FMAC->PARAM;
    reg &= ~(0x1 << 31);// mask START bit
    reg |= (0x1 << 31);// START=1
    FMAC->PARAM = reg;

    WRITE_REG(FMAC->WDATA, 0);
    while ((READ_REG(FMAC->PARAM) & (0x1 << 31)) != 0) {}


    /* set FIR */
    reg = FMAC->PARAM;
    reg &= ~(0x1 << 31);// mask START bit
    reg |= (0x0 << 31);// START=0
    FMAC->PARAM = reg;

    reg = FMAC->PARAM;
    reg &= ~((0x7F << 24) | (0xFF << 16) | 0xFF);// mask FUNC, R, P bits
    reg |= ((0x8 << 24) | (0x0 << 16) | FMAC_WINDOW);// FUNC=8(FIR), R=0, P=20
    FMAC->PARAM = reg;

    //start FIR
    reg = FMAC->PARAM;
    reg &= ~(0x1 << 31);// mask START bit
    reg |= (0x1 << 31);// START=1
    FMAC->PARAM = reg;

    //read once to start processing
    int32_t temp = READ_REG(FMAC->RDATA);

    //wait for FIR to complete
    while ((READ_REG(FMAC->SR) & 0x1) != 0) {}

    //read result to clear Y buffer
    temp = READ_REG(FMAC->RDATA);
}
```

## 質問
---質問完了---
・セッティングはここまでで、最初に一回だけ実行します。その後、各制御ループ内でWDATAに最新の角速度データを1個書き込む→フィルタ演算が終わるまで、他の処理を実行しながら待つ→終わったら結果をRDATAから読む、という流れでリアルタイムフィルタを実装します。これで大丈夫そうですか？
・今回は割り込みもDMAも使わない予定なので、FULL_WMやEMPTY_WMは0で良いと思っています。
理屈としては、FULL_WM=0ならX1バッファ内の空きが1未満(つまり0個)になるとX1FULLフラグがセットされ、空きが1個以上になるとクリアされます。X1バッファのサイズは窓幅と同じなので、新しいデータを書き込んでいいタイミングは、X1FULLフラグがクリアされているとき、というように判別できます。
また、EMPTY_WM=0ならYバッファ内の未読値が1未満(つまり0個)になるとYEMPTYフラグがセットされ、1個以上になるとクリアされます。Yバッファのサイズは1なので、RDATAから結果を読んで良いタイミング(未読値=最新の推定値が存在するタイミング)は、YEMPTYフラグがクリアされているとき、というように判別できます。
この認識であっていますか？
---質問完了---

・//start FIRの直後に、一回読み出しを行っておけばいいですか？
・各制御ループ内でも、常に最新の値を入れるとそれを使って演算が行われ、読み出すときにはその値が読みだされるというようにしたいです。1回遅れの値を読みだしたり、最新の値を入れたのにFMACが停止していて最新値による推定値が出てこなかったりしてしまわないように判定するようにしたいです。X1FULLやYEMPTYビットを使って、この判定を行うフローチャートを書いてください


## 指摘
・常に最新の推定値を得るためには、WDATAに最新データを書き込む前に、X1バッファに空きが1個ある(X1FULL==0)&Yバッファの未読値は0個(YEMPTY!=0)&MACは停止しているという条件を満たさなければならない
以下条件分けして考えます。
・YEMPTY==0の場合、MACは確実に停止していますよね？
    ・この場合、X1FULL=0であれば一回読み出すだけで上記の条件は満たせますよね？
    ・X1FULL!=0の場合は、MACは停止していますがXバッファもYバッファも満杯なので、一回RDATAを読み出してYバッファを空にし、さらに計算終了まで待ってもう一回RDATAを読み出してから、最新データを書き込む必要がありますよね？
・YEMPTY!=0かつX1FULL==0の場合、MACが停止しているならば最新データを書き込んで良いですが、MACが動作中の場合は最新データを書き込んではいけませんよね？その場合の判定はどうすれば良いですか？
・YEMPTY!=0かつX1FULL!=0の場合、MACが動作中なのは確実ですよね？現在の計算が終了するのを待ってからRDATAを読み出し、さらにもう一回計算終了まで待ってからRDATAを読み出してYバッファを空にしてから最新データを書き込む必要がありますよね？



### 指摘への回答
- **YEMPTY==0 のとき**: Y バッファ（深さ 1）が満杯なので FMAC は必ず停止しています。`RDATA` を読めば `YEMPTY` が 1 に戻り、その瞬間に X1 から 1 サンプルが消費されます。
- **YEMPTY==0 かつ X1FULL==0**: 出力だけが詰まっている状態なので `RDATA` を 1 回読み出すだけで入力にも空きができ、そのまま最新データを書き込めます。
- **YEMPTY==0 かつ X1FULL!=0**: 入出力が同時に満杯で完全に停止しています。`RDATA` を読み捨てて `YEMPTY` を 1 に戻し、演算が再開して再び `YEMPTY==0` になるまで待ってからもう 1 回 `RDATA` を読み、最新データ投入の前に X1 に空きを作ります。
- **YEMPTY!=0 かつ X1FULL==0**: 双方に空きがあっても FMAC が裏で動作中かは書き込み側だけでは判別できません。先に `WDATA` へ書き込んでから、後段の読み出し時に X1FULL を確認する運用に切り替えます。
- **YEMPTY!=0 かつ X1FULL!=0**: 出力は空でも入力が満杯なので演算中です。`YEMPTY` が 0 になるのを待って `RDATA` を 1 度読み捨てても、その時点で動き始めた演算はまだ古いデータ列を処理しています。再び `YEMPTY==0` になるまで待ってから 2 度目の `RDATA` 読み捨てを行い、X1 から 2 サンプル分を吐き出したうえで `WDATA` へ新データを書き込みます。

> 書き込みタイミングだけでは FMAC が動作中かを見分けられないため、**読み出し時に X1FULL を参照して「最新データが処理済みか」を判断する**方針にする。

- 読み出しフェーズで `X1FULL==0` なら直前に書いたサンプルまで確実に消費されています。`RDATA` から得た値をそのまま採用して良い。
- `X1FULL!=0` のままなら最新サンプルがまだ FIFO に残っている可能性があるので、`RDATA` を 1 回読み捨て → `YEMPTY` が 0 になるまで待機 → もう 1 回 `RDATA` を読み直す、という 2 段ステップで追いつかせる。



```c
int32_t latest_data = 0; // 最新の角速度データ
int32_t estimated_value = 0; // 推定された角速度

//書き込み時
void write_FMAC(int32_t data_in){
    int32_t temp = 0;
    if((FMAC->SR & (0x1)) == 0){ // YEMPTY==0
        if((FMAC->SR & (0x1 << 1)) == 0){ // X1FULL==0 : Yバッファが満杯、Xバッファは空
            // RDATAを1回読み出す
            temp = READ_REG(FMAC->RDATA);

        }else{ // X1FULL!=0 : Yバッファが満杯、Xバッファも満杯
            // RDATAを1回読み出す
            temp = READ_REG(FMAC->RDATA);
            // FMACが停止してYEMPTYが0になる(結果が出力されてYバッファが埋まる)まで待つ
            while((FMAC->SR & (0x1)) != 0){}
            // RDATAをもう1回読み出す
            temp = READ_REG(FMAC->RDATA);
        }

    }else{ // YEMPTY!=0
        if((FMAC->SR & (0x1 << 1)) == 0){ // X1FULL==0 : Yバッファが空、Xバッファも空
            // そのまま書き込める

        }else{ // X1FULL!=0 : Yバッファが空、Xバッファは満杯
            // FMACが停止してYEMPTYが0になる(結果が出力されてYバッファが埋まる)まで待つ
            while((FMAC->SR & (0x1)) != 0){}
            // RDATAを1回読み出す
            temp = READ_REG(FMAC->RDATA);
            // FMACが停止してYEMPTYが0になる(結果が出力されてYバッファが埋まる)まで待つ
            while((FMAC->SR & (0x1)) != 0){}
            // RDATAをもう1回読み出す
            temp = READ_REG(FMAC->RDATA);
            while((FMAC->SR & (1 << 1)) != 0){}// X1FULLがクリアされるまで待つ
        }
    }
    // WDATAに最新データを書き込む
    WRITE_REG(FMAC->WDATA, data_in);
}

//読み出し時
int32_t read_FMAC(){
    int32_t data_out = 0;
    while((FMAC->SR & (0x1)) != 0){} // YEMPTY=0になるまで待つ
    if((FMAC->SR & (0x1 << 1)) == 0){ // X1FULL==0
        // 正常に最新データが処理されていると判断できる
        data_out = READ_REG(FMAC->RDATA);
    } else { // X1FULL!=0
        // 最新データが処理されていない可能性がある
        // RDATAを1回読み出して捨てる
        temp = READ_REG(FMAC->RDATA);
        // FMACが停止してYEMPTYが0になる(結果が出力されてYバッファが埋まる)まで待つ
        while((FMAC->SR & (0x1)) != 0){}
        // RDATAをもう1回読み出す
        data_out = READ_REG(FMAC->RDATA);
    }
    return data_out;
}

```


i_dq_refのオーバーフロー問題
計算結果はオーバーフローしてなさそうだけど、10ビット右シフトした後もi_dq_refが32768までなのでオーバーフローしてしまう
→i_dq_refに関しては他に計算挟まないのでint_32に変更できるのでは

他の部分でもオーバーフローしそうな感じがありそうなので見直す
    掛け算の中でオーバーフローすることはなさそう

あと計算結果合わない問題
1%くらい計算値から低くなる

ゲイン
速度制御Kp : 100
速度制御Ki : 50
    50だと、速度100か1000くらいのときに,机に押し付けた時に共振して発振したので、もうちょい下げる
    30だとこの条件では発振しなかった
id制御Kp : 100
id制御Ki : 100
iq制御Kp : 100
iq制御Ki : 100

速度1000か5000くらいのときに一番ぶるぶるした気がする
これは速度制御のKiにも依存しそう
伝達関数の共振点なのかな

アンチワインドアップ
速度制御 : 100000
iq制御 : 50000