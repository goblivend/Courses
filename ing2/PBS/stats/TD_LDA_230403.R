###  1.
###  On va examiner l'analyse discriminante linéaire sur un exemple historique : les iris de Fischer
###  On comence par charger le package MASS

library(MASS)

###   2.
###   Chargement des données iris, ici la commande data(iris) fait l'affaire car les données sont incluses dans le package.
  
data(iris)

summary(iris)

## remarquer que les 3 espèces d'Iris sont bien documentées.
## on va examiner s'il est possible de les reconnaître

## 3. on lance directement la commande lda, plus tard on travaillera dans le premier plan principal

iris.lda<-lda(Species~.,iris)
summary(iris.lda)    ##  ici, pas grand chose, à par l'information que iris.lda contient des informations qu'il va falloir examiner

## 4.
## A ce stade, les calculs ont été faits avec R
## La question : ces calculs sont-ils significatifs ?
## Avant de répondre, examinons quelques résultats

## Retrouver les proportions de chacune des espèces d'iris, telles su'elles ont été conservées dans le résultat iri.lda
iris.lda$prior

## Retrouver le décompte de chacune des espèces d'iris, telles su'elles ont été conservées dans le résultat iri.lda
iris.lda$count

## Pour le total :
iris.lda$N

## Examiner les centres de gravités de chacune des espèces
## numériquement, on voit que ces centres ont l'ai d'être bien séparés
## vous comprenez ici comment une analyse en composante principale permettra plus tard de dessiner "intelligemment" les individus de notre échantillon dans le premier plan
## principal (alors, qu'en dimension 4 c'est plus difficile à représenter)
iris.lda$means

##  Pour information, retrouver les coefficients qui sont employés par R pour séparer les classes.
##  Ces coefficients ont été calculés avec la méthode présentée en cours, nous ne les recalculons pas
iris.lda$scaling

## Pour voir le résultat, garder les valeur numériques de iris dans un tableau
iris.num<-iris[,1:4]

##  5.
##  Et effectuer la prévision des classes d'après le modèle généré, utiliser la fonction predict
Prevision<-predict(iris.lda,iris.num)

## Examiner à quoi ressemble Prevision
summary(Prevision)

## 6.
## On peut alors donner la matrice de confusion et voire qu'elle est bonne
## Matrice de confusion
table(iris$Species,Prevision$class)

## 7.
##  Remarquer que les probabilites telles que présentées dans les slides sont aussi calculées
Prevision$posterior

## 8.
##  Vérifier que les probabilités calculées permettent de déduire les prévisions
Indice.Proba.Max<-apply(Prevision$posterior,1,which.max)
table(Prevision$class,Indice.Proba.Max)

## 9.
##  Maintenant, en classe, prenez des valeurs arbitraires et effectuez des prévisions


## 10. 
##  Maintenant, on va faire une ACP, puis une segmentation hiérarchique
##  Et voir la matrice de confusion obtenue ...
iris.PCA<-PCA(iris,quali.sup=5)

iris.HCPC<-HCPC(iris.PCA)
Diagn<-iris.HCPC$data.clust$clust

table(iris$Species,Diagn)     ##  On voit que la LDA marche mieux ici

### 11.
### Si vous avez le temps, reprenez l'exemple du TD sur l'ACP avec les Vins Italiens et voyez ce que cela donne
### Ici, je mets juste les commandes à effectuer 

V.It<-read.csv2("wine_italy.csv")
summary(V.It)
V.It.LDA<-lda(class~.,V.It)
Prevision<-predict(V.It.LDA,V.It)
table(V.It$class,Prevision$class)
##  Pour comparer ....
V.It.PCA<-PCA(V.It,quali.sup=1)
V.It.HCPC<-HCPC(V.It.PCA)
Diagn<-V.It.HCPC$data.clust$clust
Classe<-V.It$class
table(Classe,Diagn)

### idem pour les véhicules

Vehicules<-read.csv2("Vehicules_TD_2023.csv")
summary(Vehicules)
Vehicules.LDA<-lda(GAMME~CYL+PUIS+LON+LAR+POIDS+VITESSE,Vehicules)
Prevision.V<-predict(Vehicules.LDA,Vehicules)
table(Vehicules$GAMME,Prevision.V$class)
##  Pour comparer ....
Vehicules.PCA<-PCA(Vehicules,quali.sup=7,quanti.sup=8)
Vehicules.HCPC<-HCPC(Vehicules.PCA)
Diagn.V<-Vehicules.HCPC$data.clust$clust

table(Vehicules$GAMME,Diagn.V)

