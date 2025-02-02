###  1.
###  On va examiner l'analyse discriminante lin�aire sur un exemple historique : les iris de Fischer
###  On comence par charger le package MASS

library(MASS)

###   2.
###   Chargement des donn�es iris, ici la commande data(iris) fait l'affaire car les donn�es sont incluses dans le package.
  
data(iris)

summary(iris)

## remarquer que les 3 esp�ces d'Iris sont bien document�es.
## on va examiner s'il est possible de les reconna�tre

## 3. on lance directement la commande lda, plus tard on travaillera dans le premier plan principal

iris.lda<-lda(Species~.,iris)
summary(iris.lda)    ##  ici, pas grand chose, � par l'information que iris.lda contient des informations qu'il va falloir examiner

## 4.
## A ce stade, les calculs ont �t� faits avec R
## La question : ces calculs sont-ils significatifs ?
## Avant de r�pondre, examinons quelques r�sultats

## Retrouver les proportions de chacune des esp�ces d'iris, telles su'elles ont �t� conserv�es dans le r�sultat iri.lda
iris.lda$prior

## Retrouver le d�compte de chacune des esp�ces d'iris, telles su'elles ont �t� conserv�es dans le r�sultat iri.lda
iris.lda$count

## Pour le total :
iris.lda$N

## Examiner les centres de gravit�s de chacune des esp�ces
## num�riquement, on voit que ces centres ont l'ai d'�tre bien s�par�s
## vous comprenez ici comment une analyse en composante principale permettra plus tard de dessiner "intelligemment" les individus de notre �chantillon dans le premier plan
## principal (alors, qu'en dimension 4 c'est plus difficile � repr�senter)
iris.lda$means

##  Pour information, retrouver les coefficients qui sont employ�s par R pour s�parer les classes.
##  Ces coefficients ont �t� calcul�s avec la m�thode pr�sent�e en cours, nous ne les recalculons pas
iris.lda$scaling

## Pour voir le r�sultat, garder les valeur num�riques de iris dans un tableau
iris.num<-iris[,1:4]

##  5.
##  Et effectuer la pr�vision des classes d'apr�s le mod�le g�n�r�, utiliser la fonction predict
Prevision<-predict(iris.lda,iris.num)

## Examiner � quoi ressemble Prevision
summary(Prevision)

## 6.
## On peut alors donner la matrice de confusion et voire qu'elle est bonne
## Matrice de confusion
table(iris$Species,Prevision$class)

## 7.
##  Remarquer que les probabilites telles que pr�sent�es dans les slides sont aussi calcul�es
Prevision$posterior

## 8.
##  V�rifier que les probabilit�s calcul�es permettent de d�duire les pr�visions
Indice.Proba.Max<-apply(Prevision$posterior,1,which.max)
table(Prevision$class,Indice.Proba.Max)

## 9.
##  Maintenant, en classe, prenez des valeurs arbitraires et effectuez des pr�visions


## 10. 
##  Maintenant, on va faire une ACP, puis une segmentation hi�rarchique
##  Et voir la matrice de confusion obtenue ...
iris.PCA<-PCA(iris,quali.sup=5)

iris.HCPC<-HCPC(iris.PCA)
Diagn<-iris.HCPC$data.clust$clust

table(iris$Species,Diagn)     ##  On voit que la LDA marche mieux ici

### 11.
### Si vous avez le temps, reprenez l'exemple du TD sur l'ACP avec les Vins Italiens et voyez ce que cela donne
### Ici, je mets juste les commandes � effectuer 

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

### idem pour les v�hicules

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

