# Parte 1
1) Leggiamo la DFN e salviamo i dati nella struttura

### Determiniamo le tracce
1) Troviamo la retta di intersezione dei due piani

2) intersections = []

4) Per ogni vertice di entrambi i poligoni aggiungiamo a intersections se almeno uno dei due suoi lati intersecano la retta

5) troviamo una delle seguenti situazioni:
    - 2P: ci sono 4 punti di intersezione divisi in due coppie coincidenti
    - 3P: ci sono 4 punti di intersezione divisi in una coppia coincidente e uno rimanente
    - 4P: quattro punti di intersezione distinti
    - 5P: 4 punti distinti di cui uno è un vertice (dunque vale doppio)
    - 6P: 4 punti distinti di cui due sono vertici (dunque valgono doppio) (in questo caso il segmento è la frattura stessa)

6) calcoliamo la distanza per ogni coppia di punti

7) A seconda della situazione estraiamo una coppia di punti nei seguenti modi:
    - 2P, 3P, 4P: eliminiamo la coppia dei punti con la distanza maggiore
    - 5P: rimuoviamo il duplicato 
    - 6P: i duplicati sono la traccia

8) Determiniamo se sono passanti:
    - 2P: passante per entrambe
    - 3P, 5P, 6P: passente per una e una sola delle due (determinare quale)
    - 4P: non passante per entrambe

### Determiniamo le tracce
1) Intersechiamo ogni lato del poligono 1 con il piano del poligono 2
    - verifico se la valutazione delle coordinate è discorde: https://math.stackexchange.com/questions/2509095/intersection-between-line-segment-and-a-plane
    - se si risolvo il sistema lineare
2) se il punto sta nel poligono 2 allora lo aggiungo a intersections

Funzione che dato un poligono ritorna la normale del piano da esso generato.
Funzione che controlla se un segmento interseca un piano
Funzione che trova l'intersezione retta-piano


COSE PER LA PROSSIMA
- facciamo il conto per capire l'orientamento della proiezione per ogni intersection --> deve diventare solo per ogni frattura
- non capiamo perche la seconda intersection del primo giro viene proiettata ma essendo sul bordo considerata fuori dal poligono verde