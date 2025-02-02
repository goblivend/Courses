##
##   Ce TD utilise le package FactoMineR
##

library(FactoMineR)

##   1.
##   Vous commencez par la lecture du fichier contenant les vins fran�ais
##   Remarquer la prise en compte du fait que la premi�re colonne n'est pas une variable
##   mais d�signe le nom du vin.
##

Vins<-read.csv2("vins.csv",row.names=1)

##  2.
##  Commen�ons par "regarder" les donn�es : vous devez remarquer deux variables qualitatives, et les autres variables quantitatives.
##  Que pouvez vous dire des variables quantitatives (des notes donn�es par des experts en oenologie).

##  Vous pouvez utilisez la commande

summary(Vins)

##  3.
##  Remarquez aussi la commande fix() permettant d'explorer les donn�es, mais n'apportez pas de mordifications.

fix(Vins)

##  4. 
##  Pour ceux qui ont charg� le package corrplot, visualisez la matrice de covariance des donn�es num�riques

Corr.Vins<-cor(Vins[,-c(1,2)])    ### On fabrique la matrice de covariance sans les 2 premi�re colonnes
corrplot(Corr.Vins)
dev.new()


##  5.
##  Apr�s avoit fait un premier examen des donn�es, effectuez une analyse en composante principale des donn�es
##  Appelez la fonction effectuant l'analyse en composantes principales
##  Attention : les deux premi�re colonnes ne d�signent pas des variables num�riques.
##
##  Commencez par interpr�ter librement les deux graphiques
##  Essyez de caract�riser les deux premiers axes principaux, ici les �tudiants font une "interpr�tation libre"

Vins.PCA<-PCA(Vins,quali.sup=c(1,2))

summary(Vins.PCA)
dev.new()    ##  pour ne pas �craser les graphes ...

##  Ensuite continuez par interpr�ter les r�sultats affich�s :
##  l'attribut "Eigenvalues"
##  l'information "Individuals"      : �l�ments, Dist, Dim.n cos2, Contr est la contribution de la variable dans la formation de la dimention : plot(Vins.PCA$ind$coord[,1]^2,Vins.PCA$ind$contrib[,1],pch=19)
##  l'information "Variables"        : �l�ments Dim.n et cos2, pour visualiser les angles, on peut, si n�cessaire, refaire : plot.PCA(Vins.PCA,choix="var")
##  l'information "Supplementary categories"
##  �ventuellement regardez la documentation

##  Examinez les variances expliqu�es par les axes principaux
##

Vins.PCA$eig
DELTA<-Vins.PCA$eig[,1]
I.DELTA<-diag(DELTA)

##  I.DELTA

##  6.
##  Trouvez le moyen de visualiser les variances expliqu�es

barplot(Vins.PCA$eig[,1])

## Remarque : vous devez retrouver le nombre de variables num�riques avec la commande : sum(Vins.PCA$eig[,1])
 
sum(Vins.PCA$eig[,1])

##   7.
##   Examinez les coordonn�es des variables sur les axes principaux
##

Vins.PCA$var$coord
VARS<-Vins.PCA$var$coord 

##   Regardez la somme des carr�s des coordonn�es des composantes de VARS, que remarquez vous ??

SommeCarres<-function(v)  sum(v^2)
Resultat<-apply(VARS,1,SommeCarres)    ##  Les valeurs sont proches de 1, on ne trouve pas 1 car on a oubli� des dimensions ...

##   8.
##   Examinez les coordonn�es des individus sur les 5 premiers axes principaux
##

Vins.PCA$ind$coord
IND<-Vins.PCA$ind$coord

## 9.
## Prenez maintenant le temps d'examiner les variables sur leur graphe : appr�ciation en s�ance
## Examinez les individus et commenter librement ...
## Dessinez, sur le premier plan principal, avec trois diff�rentes couleurs, les vins Bourgueil, Chinon et Saumur
## Pouvons nous les s�parer � l'oeil ????   (r�ponse non)


I.Saumur<-which(Vins$Label=="Saumur")
I.Bourgueuil<-which(Vins$Label=="Bourgueuil")
I.Chinon<-which(Vins$Label=="Chinon")

dev.new()
plot(IND[,1:2])
points(IND[I.Saumur,1:2],pch=19,col="red")
points(IND[I.Chinon,1:2],pch=19,col="blue")
points(IND[I.Bourgueuil,1:2],pch=19,col="cyan")

##   10.
##   Maintenant nous effectuons la m�me chose avec les vins italiens
##   Avec les m�mes types de commentaires
##
##   Quelles diff�rences notable observe-t-on ???

##  r�ponse : les variables correspondent � des mesures chimiques
##            il y a 3 classes
##            les individus sont r�partis selon les classes

V.It<-read.csv2("wine_italy.csv")

### Exploration classique
summary(V.It)
corrplot(cor(V.It))

##  11.
##  Faites l'analyse en composantes principales, et commentez les axes, laisser les �tudiants faire des commentaires

V.It.PCA<-PCA(V.It,quali.sup=1)

summary(V.It.PCA)    ## Attention : il se peut que le premier axe repr�sente les tanins, le deuxi�me la couleur ....


###  12. Si on a le temps, on regarde les valeurs propres, les variables et les individus

DELTA.It<-V.It.PCA$eig[,1]
VAR.It<-V.It.PCA$var$coord
IND.It<-V.It.PCA$ind$coord
barplot(DELTA.It)

##  13. 
##  Il semble que les trois classes sont bien s�par�es

I.1<-which(V.It$class==1)
I.2<-which(V.It$class==2)
I.3<-which(V.It$class==3)

dev.new()
plot(IND.It[,1:2])
points(IND.It[I.1,1:2],pch=19,col="red")
points(IND.It[I.2,1:2],pch=19,col="blue")
points(IND.It[I.3,1:2],pch=19,col="limegreen")

##  14.
##  Nous allons maintenant effectuer une segmentation hi�rarchique des vins italiens.
##  Faites le avec 3 segments et la commande HCPC fournie dans le package.

V.It.HCPC<-HCPC(V.It.PCA)

##  15.
##  Bien examiner les trois figures obtenues  : deux dendogramme, dont un dans l'espace et une repr�sentation dans le premier plan principal, prenez le temps de regarder tout cela
##  Examiner aussi cet objet : avec R cela doit �tre un r�flexe, explorer les objets obtenus
##

summary(V.It.HCPC)
summary(V.It.HCPC$data.clust)   ##  On va se limiter � cela pour le moment

##  16.
## Trouvez ou se trouve la description des segments 
##

V.It.HCPC$data.clust$clust

##  17.
##  MATRICE DE CONFUSION
##  Comparaison entre les segments trouv�s et le classement initial des experts
##  Pour cela, fabriquer une matrice de confusion,
##  Vous pouvez utiliser la commande table

Diagn<-V.It.HCPC$data.clust$clust
Classe<-V.It$class
table(Classe,Diagn)


## 18.
## Derni�re partie du TD, r�gression lin�aire sur composantes principales.
## Nous partons de l'exemple du TD r�gression : Vehicules, 
## Pour garder le r�sultat en m�moire, nous refaisons la r�gression lin�aire du premier TD
## Puis une analyse en composante principale

Vehicules<-read.csv2("Vehicules_TD_2023.csv")
summary(Vehicules)
Modele<-lm(PRIX~CYL+PUIS+LON+LAR+POIDS+VITESSE,Vehicules) 
summary(Modele)

Vehicules.PCA<-PCA(Vehicules,quali.sup=7,quanti.sup=8)

##  19.
##  Commencer par interpr�ter les deux graphiques

##  On peut recommencer � examiner la capacit� � s�parer en 3 classes pour voir si les trois classes de v�hicules sont reconnaissables et faire une matrice de confusion

dev.new()
Vehicules.HCPC<-HCPC(Vehicules.PCA)

##  Examinez le r�sultat avec un regard critique


##  Et construisez la matrice de confusion

Diagn.V<-Vehicules.HCPC$data.clust$clust
Gamme<-Vehicules$GAMME
table(Diagn.V,Gamme)

##  20.
##  Maintenant, on va effectuer ds r�gressions sur les composantes principales, � vous de jouer !!

## Commencer par identifier les individus  puis faire la r�gresiiion
IND.V<-Vehicules.PCA$ind$coord

##  Faire plusieurs mod�les, suivant le nombre de composantes principales
Modele1<-lm(Vehicules$PRIX~IND.V[,1])
summary(Modele1)

Modele2<-lm(Vehicules$PRIX~IND.V[,1:2])
summary(Modele2)

Modele3<-lm(Vehicules$PRIX~IND.V[,1:3])
summary(Modele3)

Modele4<-lm(Vehicules$PRIX~IND.V[,1:4])
summary(Modele4)

Modele5<-lm(Vehicules$PRIX~IND.V[,1:5])
summary(Modele5)

##  Essayer, librement pour les �tudiants, de r�duire le nombre de composantes principales permettant d'expliquer le prix du v�hicule

Modele6<-lm(Vehicules$PRIX~IND.V[,c(1,3,4,5)])      ##  Ici, je suis surpris
summary(Modele6)

plot.PCA(Vehicules.PCA,axes=c(1,5),choix="ind")   ##  Accessoire, je ne commente pas

Modele7<-lm(Vehicules$PRIX~IND.V[,c(1,5)])      ##  Ici, je suis surpris
summary(Modele7)

####  21. Et si vraiment il reste du temps !!!
####  Donn�es Boston Essayez de reprendre la r�gression du premier cours, mais cette fois sur les composantes principales


Boston<-read.csv2("Donnees_Boston.csv")
summary(Boston)

dev.new()
Boston.PCA<-PCA(Boston,quanti.sup=13)

IND.B<-Boston.PCA$ind$coord

Modele.B0<-lm(medv~.,Boston)
summary(Modele0)

## Ici nous laissons les �tudiants essayer plusieurs mod�les  ...

Modele.B1<-lm(Boston$medv~IND.B[,1])
summary(Modele.B1)

Modele.B5<-lm(Boston$medv~IND.B[,1:5])
summary(Modele.B5)

