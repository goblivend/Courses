###   TD sur la régression linéaire

####  1. Lecture des données

####  Commencez par vous placer dans le bon répertoire  avec R

dir() ## pour vérifier

#### Lisez les données  à partir du fichier Vehicules_TD_2023.csv
Donnees<-read.csv2("Vehicules_TD_2023.csv")

####  2. Prenez le temps d' "observer" en direect les données :
####  2.1
####   y a-t-il des valeurs manquantes ?
####   y a-t-il des valeurs aberrantes ?

#### 2.2
#### Faites l'exercice qui a été fait en cours d'examiner les données
#### avec les même fonctions 
#### si vous avez chargé le package corrplot utilisez corrplot

summary(Donnees)               ##  Pas de valeurs manquantes

dev.new()
boxplot(Donnees$CYL,main="CYL")
dev.new()
boxplot(Donnees$PUIS,main="PUIS")
dev.new()
boxplot(Donnees$LON,main="LON")
dev.new()
boxplot(Donnees$LAR,main="LAR")
dev.new()
boxplot(Donnees$POIDS,main="POIDS")
dev.new()
boxplot(Donnees$VITESSE,main="VITESSE")
dev.new()
boxplot(Donnees$PRIX,main="PRIX")

#### 2.3  Regardez la matrice de corrélation, essayer de la visualiser

M<-cor(Donnees[,-7])
plot(Donnees[,-7])
corrplot(M) 


#### 3. Un peu d'esprit critique : quelle question peut-on se poser ?
#### Les trois catégories de véhicules sont elles "visualisables"
#### Examinez les centres de gavité des différnets catégories
## On y va !
 
Ind.BASSE<-which(Donnees$GAMME=="BASSE")
Ind.MOYENNE<-which(Donnees$GAMME=="MOYENNE")
Ind.HAUTE<-which(Donnees$GAMME=="HAUTE")

summary(Donnees[Ind.BASSE,])
summary(Donnees[Ind.MOYENNE,])
summary(Donnees[Ind.HAUTE,])

 
####  5. La régression linéaire : Effectuez maintenant la régression lnéaire de la variable PRIX par rapport aux autres variables numériques

Modele<-lm(PRIX~CYL+PUIS+LON+LAR+POIDS+VITESSE,Donnees) 

summary(Modele)

#### 4.1
####   Exploration de chacun des attributs, pour exploitation ultérieure  et vérification de quelques formules du cours

#### Commençons par les coefficients
#### Seriez-vous capable de les retrouver à partir de la formule matricielle du cours ?
#### Etes vous capables de calculer la prévision d'une valeur  (par exemple d'une valeur associée aux données fournies) ?
Modele$coefficients

####   4.2
####   Essayons de retrouver ces coefficients avec la formue du cours

X<-cbind(rep(1,18),as.matrix(Donnees[,1:6]))
Y<-Donnees$PRIX
A<-solve(t(X) %*% X) %*% (t(X) %*% Y)

#### Comparez A avec Modele$coefficients 

#### 4.3
####   Maintenant in regarde les valeurs prévues calculées par R ?

Modele$fitted.values
Exemple<-Donnees[7,]
Pred.Prix<-predict(Modele,Exemple)
#### Comparer avec 
Pred.Prix
Modele$fitted.values[7]

#### Et faisons les calculs avec les coefficients

##### Perso, j'ai eu un problème avec les types de données, car Donnees est un data frame : il y a aussi des données non numériques
##### J'ai fait basique

Valeur.Num.Exemple<-c(1,as.numeric(Exemple[1,1:6]))
Prix.Calcule <- sum(Modele$coefficients * Valeur.Num.Exemple)

#### Et comparez : 

Prix.Calcule
Modele$fitted.values[7]

#### Dessinez le graphe prix / valeurs prévues	
plot(Donnees$PRIX,Modele$fitted.values)

#### 4.4
####  Maintenant on va regarder les résidus
#### Vérifiez en direct qu'il s'agit bien de résidus (un exemple suffit)
Modele$residuals

Exemple<-Donnees[7,]
Pred.Prix<-predict(Modele,Exemple)
Pred.Prix
Exemple$PRIX-Pred.Prix
Modele$residuals[7]
       

#### Les résidus ont il un comportement de loi normale, vous avez le droit d'être subjectif

qqnorm(Modele$residuals)
m.residuals<-mean(Modele$residuals)
sd.residuals<-sd(Modele$residuals)

CR.residuals<-(Modele$residuals - m.residuals)/sd.residuals

mean(CR.residuals^2)
mean(CR.residuals^3)
mean(CR.residuals^4)

#### 4.5
#### Maintenant, vous allez choisir des "individus" différnets avec des paramètres plausibles 
#### Voire les prévisions que donnent le modèle !
#### A vous de jouer !

## Il suffit de modifier l'exemple

Exemple$CYL<-1500
Exemple$PUIS<-100 
Exemple$LON<-420 
Exemple$LAR<-175 
Exemple$POIDS<-977 
Exemple$VITESSE<-175
Pred.Prix<-predict(Modele,Exemple)
Pred.Prix 

#### 5.
#### Avant d'interpréter les autres éléments du modèle, vous allez, en groupe
#### constater que les coefficients sont imprécis
#### Pour cela, vous allez chacun de vous reprendre le tableau de données et
#### supprimer une ligne différente
#### refaire le modèle linéaire sur le nouveau tableau
#### et comparer entre vous les différentes valeurs des coefficients

#### Un exemple, ici je supprime la ligne 5, évidemment, les étudiants doivent en supprimer d'autres pour qu'il y ait le plus de modèles possibles !!

Ligne.Sup<-5

Donnees.Nouvelles<-Donnees[-5,]
Modele.Nouveau<-lm(PRIX~CYL+PUIS+LON+LAR+POIDS+VITESSE,Donnees.Nouvelles)  
summary(Modele.Nouveau)


#### 6.
#### Maintenant, on va examiner le caractère significatif des coefficients
#### examinez chaque composante du résultat summary(Modele)
#### et commentez.



#### Si vous deviez choisir un modèle avec moins de variables, comment feriez vous ???
### à vous de jouer, je vous laisse faire, il faurt laisser un peu de liberter aux étudiants !!!

### Un exemple ...

Modele2<-lm(PRIX~CYL+PUIS+LON+LAR,Donnees) 
summary(Modele2)

#### 7.
#### Faites une régression en séparant un ensemble d'apprentissage et en ensemble de test
#### Comparez le R2 calculé par R avec l'erreur quadratique que vous avez obtenue
#### Commentez !

#### Comme il y a 18 données, on va en prendre les 2/3 au hasard pour construire le modèle, soit 12, et le reste pour comparer le résultat

I.Modele<-sort(sample(18,12))
I.Test<-setdiff(1:18,I.Modele)

Donnees.Apprentissage<-Donnees[I.Modele,]
Donnees.Test<-Donnees[I.Test,]

Modele.3<-lm(PRIX~CYL+PUIS+LON+LAR+POIDS+VITESSE,Donnees.Apprentissage)
previsions.Test<-predict(Modele.3,Donnees.Test)

var(Modele$residuals)
var(Donnees.Test$PRIX-previsions.Test)     ### En principe devrait être supérieur à ce qui précède, mais je suis déjà tombé sur des exceptions

plot(Donnees.Test$PRIX,previsions.Test)

#### 8.
#### Même exercice avec d'autres données je vous propose census_data
#### Je reprends les commandes essentielles, avec un minimum de commentaire
#### Vous remarquerez que je ne refais pas tout

Census<-read.csv2("Census_data_fr.csv")

summary(Census)

MS<-cor(Census)
corrplot(MS)        ## Commentaire libre entre vous

#### et construisez un modèle permettant de prédire la variable Life.Exp

Modele.LifeExp<-lm(Life.Exp~.,Census) 

summary(Modele.LifeExp)     #### Commentez librement le signe des coefficients, remarquez le R2 d'environ 74% 

plot(Census$Life.Exp,Modele.LifeExp$fitted.values,pch=19,cex=0.8,col="blue")    ## un peu de couleur pour changer

qqnorm(Modele.LifeExp$residuals,pch=19,col="red")



#### Essayez librement de réduire le nombre de variables du modèle,
#### Garder  Murder + HS.Grad + Frost donne de bons résultats  : Modele.3vars<-lm(Life.Exp~Murder + HS.Grad + Frost,Census)
#### Soyez libre de commenter, et laissez les étudiants hésiter un petit peu avant de proposer ce modèle
#### A vous de jouer !!!







