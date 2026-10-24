# DipLabSim — Double Inverted Pendulum Lab

C# / .NET 10 / WPF masaüstü simülatörü. İki bağımsız doğrusal olmayan model,
dört solver ve manuel açık çevrim kuvvet girişi içerir.

## Windows'ta çalıştırma (VS Code yeterli)

1. [.NET 10 SDK, Windows x64](https://dotnet.microsoft.com/download/dotnet/10.0) kurun (Runtime değil SDK).
2. VS Code'u yeniden açın. Terminalde `dotnet --list-sdks` ile `10.0.xxx` göründüğünü doğrulayın.
3. Bu dalı indirin/clone edin; terminali repo kökünde açın.
4. Çalıştırın:

```powershell
dotnet run --project DipLabSim.UI/DipLabSim.UI.csproj
```

İlk restore için internet gerekir. Visual Studio zorunlu değildir. WPF penceresi
Windows'ta açılır; GitHub Pages'te veya tarayıcıda çalışmaz.
GitHub Actions başarılı olduğunda ilgili run'ın Artifacts bölümündeki
`DipLabSim-Windows-x64` paketini indirip ZIP'i tamamen çıkartarak
`DipLabSim.UI.exe` dosyasını SDK kurmadan da çalıştırabilirsiniz.

## İlk deneme

- Varsayılan RK4, dt = 0.001 s ve küçük başlangıç açılarıyla **Play**: kontrolsüz sarkaç devrilir.
- **Pause**: model, solver, dt ve fiziksel parametreler düzenlenebilir.
- **Initial State**: x, θ₁, θ₂ başlangıç konumları; hızlar sıfırdır.
  **Reset** bu ayarlara ve t = 0'a döner. **Upright** tam dik dengedir;
  ideal aritmetikte dış giriş olmadan düşmesi beklenmez. **Perturbed** düşüşü gösterir.
- **Forces**: arabaya ±30 N aralığında sürekli kuvvet verin; **Stop force** ile sıfırlayın.
  Cart ← / → butonları aşağıdaki kuvvet büyüklüğünü kullanır.
- **LowerLinkTip**: alt çubuğun üst ucu / iki çubuğun ortak eklemi.
  **UpperLinkTip**: üst çubuğun serbest ucu.
- Play sırasında hedefi, büyüklüğü ve süreyi seçip **Push ← / →** ile yatay kuvvet darbesi uygulayın.
  Bu, kuvvet [N] × süre [s] ile tanımlanan dikdörtgen darbedir; açıyı elle sıçratmaz.
  Süre dt'nin tam katına yukarı yuvarlanır; yeni darbe eskisinin yerini alır.
- Kırmızı ok bozucunun uygulandığı ucu ve yönünü gösterir. Grafikler x, mutlak açılar,
  araba kuvveti ve link bozucusunu gösterir. Açılar hesapta sarılmaz.
- Sayısal taşma / geri Euler yakınsamama durumunda simülasyon durur ve durum çubuğu
  nedeni gösterir. dt'yi veya kuvveti azaltıp Reset uygulayın.

## Modeller ve solverlar

Her iki model aynı kütle, geometri, COM ataleti, yerçekimi ve sürtünme parametrelerini
kullanır. Model seçimi sırasında parametreler korunur. Varsayılanlar rapor 2 tablosudur:
M=1.5, m1=0.5, m2=0.75 kg; L1=0.5, L2=0.75 m; l1=0.25, l2=0.375 m;
J1=0.010417, J2=0.035156 kg·m²; g=9.81 m/s². Varsayılan sürtünme sıfırdır.
Uzunluk/kütle düzenlemek ataleti otomatik yeniden hesaplamaz; J ve COM bağımsız girdilerdir.
COM mesafesi çubuk uzunluğunu aşamaz; çubuk kısalırsa COM buna sınırlandırılır.

- **Euler–Lagrange**: enerji türetiminden simetrik 3×3 kütle sistemi.
- **Newton–Euler**: her çubuğun COM ivmesi, kuvvet ve moment dengeleri;
  eklem reaksiyonları bağımsız olarak elenir. Model 1'in türev fonksiyonunu çağırmaz.
- **Forward Euler**: 1. mertebe, açık yöntem.
- **Heun**: 2. mertebe açık predictor/corrector.
- **RK4**: 4. mertebe, varsayılan açık yöntem.
- **Backward Euler**: 1. mertebe örtük yöntem; sönümlü Newton iterasyonu ile
  `y - x - dt*f(t+dt,y) = 0` çözülür. Yakınsamazsa hata bildirir.

Büyük dt özellikle ileri Euler'de yapay enerji artışı oluşturabilir; geri Euler
sayısal sönüm ekleyebilir. Devrilmenin fiziksel olması tek başına solver doğruluğu kanıtı değildir.

Koordinatlar: `q=[x, θ1, θ2]`, her iki açı düşey yukarıdan ölçülen **mutlak** açıdır;
pozitif x / kuvvet sağa, pozitif açı sağa yatıştır. Üst eklem sürtünmesi
`c2*(ω2-ω1)` göreli hızına etkir. Hedef uçtaki yatay F bozucusu, sanal işten:

- Alt uç: `Qdist=[F, F*L1*cos(θ1), 0]`.
- Üst uç: `Qdist=[F, F*L1*cos(θ1), F*L2*cos(θ2)]`.

Bu nedenle link bozucusu arabanın hareketini de etkiler. Sistemde eklem motoru yoktur.
Ray sonu çarpışması ve yerle temas modellenmez; çubuklar tam dönüş yapabilir.

## Görseldeki mimari

- `DipLabSim.Core`: dış paket bağımlılığı olmayan ortak sözleşmeler.
- `DipLabSim.Physics`: iki model + dört solver; yalnızca Core'a proje referansı.
- `DipLabSim.Engine`: sabit dt ile arka plan thread'i; yalnızca Core'a proje referansı.
- `DipLabSim.UI`: composition root, MVVM, SkiaSharp, ScottPlot; WPF adaptörü
  bounded `Channel<SimulationFrame>` üzerinden son kareyi render hızında okur.

Fizik dt'si ekran yenilemesinden bağımsızdır. Hız çarpanı adım büyüklüğünü değil,
gerçek zamana göre atılan adım sayısını değiştirir. Parametre düzenlemek için thread durdurulur.
Windows üzerinde hard real-time garantisi yoktur; uzun OS duraklamasında catch-up sınırlandırılır.
Grafikler render hızında örneklenir; tam 1 kHz veri kaydı değildir. Çok kısa kuvvet darbeleri
grafikte görünmeyebilir ama fizik adımlarına uygulanır. Grafik belleği sınırlı tutulur.

Görseldeki `Extensibility`, DLL/gRPC kontrolcü çalıştırma ve 200 Hz kapalı çevrim
kontrol döngüsü bu açık çevrim tesliminde henüz uygulanmamıştır. Mevcut controller
scanner gelecekteki entegrasyon için korunmuştur; arayüz bu sürümde açık çevrimi gösterir.

## Doğrulama

```powershell
dotnet test DipLabSim.Numerics.Tests
dotnet test DipLabSim.Tests
```

Numerics testleri Windows UI gerektirmez: iki modelin rastgele durum/parametrelerde
türev eşdeğerliği; enerji-güç dengesi (iki bozucu hedefi ve sürtünmeyle); 5 saniye
enerji korunumu; analitik osilatörde solver yakınsama mertebeleri; örtük yöntem
başarısızlığı; sıfır heap allocation; darbe süresi, reset ve worker pause/resume.
UI testleri Windows ve gerçek STA thread kullanır. CI hem Linux numerics hem Windows
build/UI testleri çalıştırır. MATLAB referansı verilmediği için MATLAB'a karşı
`<1e-6` doğrulama iddiasında bulunulmaz; hocanın model düzeltmeleri henüz bilinmemektedir.
