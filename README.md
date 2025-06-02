**Zadanie 3 (15b)**
V tomto zadaní budete vytvárať riadenie pre mobilný robot Kobuki (real/sim). Práca so simuláciou a simulovaným robotom je popísaná v prezentáciách na cvičeniach 8.

**Samotné simulačné prostredie:**
![image](https://github.com/user-attachments/assets/2bee4a5a-35a2-49c6-9599-6f93aca2cbc6)

Znenie zadania
Robot má mať vytvorený riadiaci program v ROS2 tak, že bude pozostávať z nasledujúcich režimov.
**
Režimy:
**
**Vypnutý stav. **Do robota nejdú žiadne riadiace príkazy z programu. V tomto režime bude začínať program. Pri prechode zo zapnutého do vypnutého režimu treba robota dať do stavu vhodného na vypnutie (zastavenie a skontrolovanie úspešného zastavenia).

**Zapnutý stav a manuálny režim.** Robot bude možné ovládať pomocou manuálnych tlačidiel. Bude sa dať ovládať lineárna a rotačná rýchlosť robota a taktiež sa bude dať ho okamžite zastaviť. Po prechode z automatického do manuálneho režimu je plynulé pokračovanie riadiacich príkazov (ak má aktuálnu lineárnu rýchlosť 0.5 a prírastok je 0.1, tak znížením rýchlosti tlačidlom sa nastaví nová lineárna rýchlosť 0.4).

**Zapnutý stav automatický režim.** Robot bude umiestnený do ľubovoľnej pozície a orientácie v prostredí. Robot sa pohne priamo dokým sa nepriblíži dostatočne blízko k stene (bez narazenia alebo prekročenia určenej vzdialenosti). Keď sa priblíži k stene, natočí sa tak, aby dokázal sledovať danú stenu v určitej vzdialenosti zadanej v metroch ako vstup do programu (napr.: 0.3m). To či sa natočí v smere alebo v protismere hodinových ručičiek nie je podstatné. Na rozpoznanie steny použite Lidar. Stena môže byť rôzne členitá a rohy nemusia mať pravé uhly (viď Obr. 1). Na Obr. 1 sú znázornené aj rôzne akceptovateľné spôsoby otáčania sa okolo rohu a prechodu pozdĺž steny. Dôležité je, aby robot nenarazil do steny.

Voľba čísla režimu bude zadaná ako vhodný ROS2 topic. Režimy sa budú dať meniť aj počas behu simulácie.

Počas celého behu simulácie sa bude počítať prejdená dráha robota, čo vyveďte do vhodného ROS topicu a zobrazujte v grafe (matplotlib). Zmenou režimu sa neresetuje prejdená dráha.

**Manuálne ovládanie:**

![image](https://github.com/user-attachments/assets/766cfd02-f479-4dee-95e4-a8c727db077c)



**Rýchly manuál na celé ovládanie zadania:**
1.	Vytvoriť si niekde ros2_ws, do src vložiť priečinok zadanie3
2.	Zapnúť Kobubi Simulator, v ňom klik na START ROBOT
3.	Zapnúť komunikačné rozhranie UDS Kobuki ROS
4.	Otvoriť x64 Native Tools Command Prompt for VS 2022 kde cez vložiť príkazy 
c:\opt\ros\foxy\x64\setup.bat
cd c:\cesta k ros balíčku\ros2_ws
call install/setup.bat
colcon build --merge-install --packages-select zadanie3
ros2 run zadanie3 zadanie
5.	Po príkaze ros2 run by sa mal zapnúť program, defaultne v OFF móde, voľba režimu
6.	Pri voľbe manuálneho režimu (stlač 1) ovládanie cez šípky a tlačítka, viď. riadenie manuálny režim
7.	Pred voľbou automatiky (stlač 2) nastaviť linear na 0.4, angular 0.2, prísť s robotom na požadované miesto
8.	Spustiť automatiku a sledovať pohyby
9.	Cez klávesu 0 možnosť dať OFF mód, resp. vypnutie celej aplikácie 

