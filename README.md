# GardenMowerAutopilot
My final project for informatics.

# Ugotvaljanje kateri sistem za prepoznavanje predmetov je najbolj primeren.
1. Uporabljam MobileNetSSD
2. Premik na yolo, zaradi višje natančosti za pse, mačke in ljud, kar je ključnega pomena pri varnosti kosilnice.
3. Premik nazaj na mobilenetssd zaradi pomena hitrosti(rpi je fuuuul počasn, če ni ai kamere)
(Opiši tudi kako si optimiziral, da je video stream z zelo majhno zakasnitvijo. Najprej se je slika posodobila vsako 1,5s, potem sem zmanjšal resolucijo in natačnost modela, ampak testiral, da je dovolj dobro da se ustavi ko zazna človeka)

# Ugotvaljanje delovanja sistema za nadzor na daljavo
Poiskal sem dokumentacijo za sprejemnik microzone receiver mc8re http://microzone.cn/notes/MC8RE-V2/MC8RE-V2%E8%AF%B4%E6%98%8E%E4%B9%A6%EF%BC%88%E8%8B%B1%E6%96%87%EF%BC%89.pdf
Z Raspbery piem sem izmeril kaj se zgodi na pinu, ki oddaja pwm signal ko na daljincu pritisnem joystick naprej, nazaj, levo, desno. Ugotovil sem to:
Naprej: gpio27 2000, gpio17 1050
Pri miru: gpio27 in  gpio17 1500
Nazaj: gpio27 1050, gpio17 2000
Levo: gpio17 in 27 na 2000
Desno: gpio17 in 27 na 1050

potem pa sem opazil, da lahko uporabim še kakšen drug kanal za dodatno funkcijo, zato sem se odločil, da bom CH5 uporabil za določanje ali je varnost(zaustavitev ob zaznavi ovire) vklopljena ali ne.
Odločil sem se, da bom v mojem izdelku uporabljal te vrednosti, da določilm način delovanja.
<img width="413" height="397" alt="image" src="https://github.com/user-attachments/assets/7408ac78-76ad-4fd8-8481-10e025889694" />
Gpio22 za delček sekunde na 1000: vklop varne vožnje
Gpio22 za delček sekunde na 2000: izklop varne vožnje

# Izdelava spletnega vmesnika za lažji nadzor:
Dodal sem tudi server, ki ga ima raspbery pi in lahko do njega dostopamo, če smo povezani na isto omrežje.

(Fotografija UIja)
<img width="1918" height="1025" alt="image" src="https://github.com/user-attachments/assets/fe93c20f-79c5-4761-9b62-7fab924e6dc3" />

# Problem z gps
Ker gre pri košenju za opravilo, kjer je potrebna velika natačnnost, sem želel dodati tudi gps, da bo operater lažje vedel kje se nahahaja, ko bo kosil preko daljnica. Ker običajni gps senzorji kot so(naštej jih nekaj) niso dovolj natačni sem kupil: lc29h, ki ponuja dgps(pojasni kaj je to). Preko ntip strežnik https://rtk2go.com/ dobivam popravke s postaje NTRIP_HOST = "rtk2go.com"
NTRIP_PORT = 2101
MOUNTPOINT = "FRELIH"  
	Krize, Slovenia

  teste sem izvajal v Cerknem, ki je od nje oddaljeno cca. 33km in je dovolj blizu, da so popravki še dovolj natačni. Glede na teste je modul natančen do 1m, kar je veliko boljše kot cenejši gps moduli(npr. .. ), ki dosežejo natančnost +-5m, ki se kdaj nenadoma spremeni v 30m ali celo več.

  
