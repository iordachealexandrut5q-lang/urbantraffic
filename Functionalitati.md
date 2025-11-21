# Funcționalități pentru tema: Simularea traficului urban – Urban Traffic Simulator

============================================
MUST HAVE
=========================================================

## I. Infrastructura de bază a orașului

1. Generarea orașului
   -> Crearea automată a unei rețele de străzi și intersecții într-o structură rectangulară.
   -> Poziționarea intersecțiilor în funcție de dimensiunea ferestrei de simulare.
   -> Conectarea nodurilor adiacente prin drumuri bidirecționale.
   -> Posibilitatea de a configura dimensiunea orașului (număr de rânduri și coloane).

2. Gestionarea drumurilor
   -> Activarea și dezactivarea drumurilor prin interacțiune directă.
   -> Actualizarea automată a rutelor traficului în funcție de starea fiecărui drum.
   -> Evidențiere vizuală a străzilor active și a celor blocate.
   -> Reprezentare grafică realistă a lățimii drumurilor.

3. Gestionarea intersecțiilor
   -> Afișarea intersecțiilor sub formă de noduri vizuale pentru debugging și analiză.
   -> Posibilitatea de a selecta intersecții pentru vizualizarea unei rute optime.
   -> Previzualizarea traseului ideal între două puncte selectate.

-------------------------------------------------------------------------------

## II. Dinamica traficului și rutare
1. Rutare automată
   -> Calcularea traseului optim între două puncte din oraș.
   -> Rerutarea dinamică a vehiculelor atunci când un drum devine inaccesibil.
   -> Adaptarea continuă a traficului în funcție de condițiile actuale ale rețelei.

2. Simularea vehiculelor
   -> Generarea automată a unui număr prestabilit de vehicule în oraș.
   -> Deplasare realistă pe drumuri, cu menținerea unei distanțe de siguranță între mașini.
   -> Viteză variabilă pentru fiecare vehicul, pentru diversitate în comportament.
   -> Reversarea traseului după atingerea destinației, simulând deplasarea zilnică.

3. Timp și ciclu zi/noapte
   -> Simulare a timpului din zi, cu viteză ajustabilă a trecerii orelor.
   -> Iluminare ambientală dinamică în funcție de oră (zi, seară, noapte).
   -> Afișarea orei curente în interfață pentru urmărirea fluxului zilnic.

-------------------------------------------------------------------------------

## III. Interfață vizuală și interacțiune

1. Interfață grafică
   -> Afișare în timp real a vehiculelor, drumurilor și intersecțiilor.
   -> Actualizare constantă a pozițiilor și culorilor elementelor din simulare.
   -> Indicarea vizuală a intensității traficului pe fiecare drum.
   -> Suport pentru interacțiuni cu mouse-ul pentru modificarea mediului.

2. Previzualizarea rutelor
   -> Posibilitatea de a selecta două intersecții și de a vizualiza ruta optimă.
   -> Util pentru analizarea comportamentului sistemului de rutare și testarea conexiunilor.

============================================
                   NICE TO HAVE
===========================================================

## I. Extensii de infrastructură și simulare

1. Hărți de căldură
   -> Afișarea gradului de aglomerație în timp real printr-o hartă de intensitate colorată.
   -> Zonele cu trafic ridicat sunt evidențiate vizual pentru analiză urbană.

2. Semne de oprire și prioritate
   -> Implementarea punctelor de oprire în intersecții fără semafor.
   -> Gestionarea priorității între vehicule în funcție de direcția de mers.

3. Drumuri cu mai multe benzi
   -> Extinderea drumurilor pentru a permite circulația pe mai multe benzi.
   -> Comportament diferențiat al vehiculelor în funcție de bandă.

4. Zone rezidențiale și zone de lucru
   -> Definirea unor zone din oraș drept cartiere rezidențiale (start) și zone de birouri/industriale (destinații).
   -> Corelarea deplasărilor zilnice cu ciclul zi-noapte, simulând un program de muncă.
   -> Aflux de trafic dimineața spre zona de lucru și seara spre rezidențială.

5. Transport industrial
   -> Simulare de vehicule de marfă care transportă resurse între oraș și zonele industriale.
   -> Activitate crescută în intervalele orare specifice livrărilor.

6. Limite de viteză diferențiate
   -> Stabilirea unor limite de viteză specifice pentru diferite tipuri de drumuri.
   -> Vehiculele adaptează automat viteza în funcție de zona traversată.

7. Puncte de interes (Points of Interest)
   -> Adăugarea de locații speciale (școli, parcuri, magazine, fabrici).
   -> Generarea de fluxuri suplimentare de trafic către aceste puncte în funcție de oră.

-------------------------------------------------------------------------------

## II. Elemente de trafic și infrastructură avansată

1. Sensuri unice și sensuri giratorii (Roundabouts)
   -> Implementarea de intersecții circulare cu flux controlat.
   -> Optimizarea traficului în nodurile cu volum mare.

2. Transport public
   -> Introducerea autobuzelor și tramvaielor cu rute fixe.
   -> Reducerea numărului de vehicule personale prin creșterea capacității transportului public.

3. Sistem de accidente și blocaje
   -> Generarea aleatorie a incidentelor care blochează temporar drumurile.
   -> Adaptarea rutelor vehiculelor pentru evitarea zonelor afectate.

4. Vehicule de urgență
   -> Ambulanțe, pompieri și poliție cu prioritate în trafic.
   -> Deplasare liberă prin trafic și comportament de urgență în caz de incident.

5. Sistem de populație
   -> Corelarea numărului de vehicule și mijloace de transport public cu populația orașului.
   -> Evoluție dinamică a populației în funcție de densitatea traficului și infrastructură.

6. Stații de alimentare (Benzinării)
   -> Introducerea punctelor de realimentare pentru vehicule.
   -> Planificarea rutelor în funcție de nivelul de combustibil.

-------------------------------------------------------------------------------

## III. Analiză, raportare și interfață extinsă

1. Meniu de configurare
   -> Posibilitatea de a seta dimensiunea orașului, numărul de vehicule, viteza simulării și alte opțiuni.
   -> Pornire, oprire și resetare a simulării din interfață.

2. Statistici și rapoarte
   -> Afișarea numărului de vehicule active, a nivelului de congestie și a timpului mediu de deplasare.
   -> Exportarea datelor în format text sau tabelar pentru analiză.

3. Vizualizare interactivă
   -> Zoom, panoramare și selecție de obiecte în hartă.
   -> Afișarea traseului și a informațiilor individuale pentru fiecare vehicul.
