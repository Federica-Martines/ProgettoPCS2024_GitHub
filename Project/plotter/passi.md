### creo mesh iniziale

1) per ogni traccia cerco intersezioni con ogni lato e ritorno un {coord, alpha, lato}
2) ordino questo vettore per alpha e estraggo quelli <= a 1 e il primo < 1 e il primo > di 1
3) per ogni coppia di intersezioni con la seconda delle due che ha alpha < 1 spacco le due celle
    mentre per per quelle con alpha > 1 spacco le celle e aggiorno il vicino del lato 

SplitCell2D(cell2D, intersections)
    SplitEdges(intersections)

    CreateSubCells


1) per ogni traccia cerco la sua Cella2D di appartenenza
2) interseco il taglio con ogni lato della cella2D
3) splitto il lato e la cella in 2
4) Per entrambe le intersezioni trovate se l'abs di alpha è > 1 ripeto con il vicino del lato 
5) Se è <= a 1 aggiorno il vicino 

