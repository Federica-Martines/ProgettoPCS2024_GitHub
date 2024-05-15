# Parte 1
1) Leggiamo la DFN e salviamo i dati nella struttura

### Determiniamo le tracce
1) Troviamo la retta di intersezione dei due piani

2) intersections = []

3) Se un vertice dei due poligoni appartiene alla retta lo aggiungiamo a intersections

4) Intersechiamo la retta con tutti i lati di entrambi i poligoni (a eccezione di quelli gia esclusi con i vertici)

5) troviamo le seguenti situazioni:
    - 2P: ci sono 4 punti di intersezione divisi in due coppie coincidenti
    - 3P: ci sono 4 punti di intersezione divisi in una coppia coincidente e uno rimanente
    - 4P: quattro punti di intersezione distinti
    - 5P: 4 punti distinti di cui uno è un vertice (dunque vale doppio)
    - 6P: 4 punti distinti di cui due sono vertici (dunque valgono doppio) (in questo caso il segmento è la frattura stessa)