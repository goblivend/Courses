
setwd("D:/gits/Courses/ing2/PBS/stats")

vehicules <- read.csv2("Vehicules_TD_2023.csv")

summary(vehicules)

boxplot(vehicules$POIDS, main="POIDS")

Matrix <- cor(vehicules[,-7])
plot(vehicules[,7])
library("corrplot")
corrplot(Matrix)
# Toutes les variables sembkent corrélées entre elles, il va falloir toutes les inclures dans le modèle 

# 3 
# On a envie d'expliquer le prix en utilisant les autres variables

gamme_basse <- vehicules[vehicules$GAMME=="BASSE",]
gamme_moyenne <- vehicules[vehicules$GAMME=="MOYENNE",]
gamme_haute <- vehicules[vehicules$GAMME=="HAUTE",]

summary(gamme_basse)
summary(gamme_moyenne)
summary(gamme_haute)
# Hypothèse : Gamme reflète le prix, possibilité de l'enlever
# => Regression du prix sur toutes les autres variables

# 4 

# 4.1

modele <- lm(PRIX~CYL+PUIS+LON+LAR+POIDS+VITESSE, data=vehicules)
summary(modele)
# on regarde les résidus : on regarde si la médianne est proche de 0 et si la repartition semble uniforme : semble ok

# Ensuite on regarde les coefs : la première colonne donne la valeur estimée du coef, la dernière donne la significativité de la variable, on teste si le coef est à 0, et on renvoit la p-value du test. On garde en général les variables significatives à 5% et moins.
# on regarde le R^2 et le R^2 ajusté (il tient compte du nombre de variables du modèle)
# On les veut le plus proche possible de 1 (pas de seuil mais au dessus de 0.7 => très bon modèle). le R^2 sert surtout à comparer 2 modèles et garder le meilleur.

modele$coefficients

X <- cbind(rep(1, 18), as.matrix(vehicules[,1:6]))
Y <- vehicules$PRIX
A <- solve(t(X) %*% X) %*% (t(X) %*% Y)

# 4.3
modele$fitted.values
example <- vehicules[5,]
prix_predit <- predict(modele, example)
modele$fitted.values[5]
example$PRIX

# On a une prediction qui n'est pas très éloignée du vrai prix
plot(vehicules$PRIX, modele$fitted.values)

# Le but est de comparer ce graphe  à la droite d'equation y=x qu'on aurait si le prix prédit était égal au vrai prix

# 4.4
modele$residuals

example$PRIX - prix_predit
qqnorm(modele$residuals)
# Pas de règle stricte, au cas pas cas : on regarde si le graph nous donne des points répartis sur une droite passant par l'origine.
# Ici, on va se dire que les résidus suivent bien une loi normale.

sd(modele$residuals)
# On peut calculer des résidus centrés réduits
CR_residuals <- (modele$residuals - mean(modele$residuals))/sd(modele$residuals)
plot(CR_residuals)
# on peut ensuite travailler sur ces résidus

# 4.5
example$CYL <- 1500
example$PUIS <- 90
example$LON <- 430
example$LAR <- 162
example$POIDS <- 1000
example$VITESSE <- 150
prix_predit <- predict(modele, example)  
prix_predit

# 5

data_supp <- vehicules[-5,]
modele2 <- lm(PRIX~CYL+PUIS+LON+LAR+POIDS+VITESSE, data=data_supp)

# compare
modele
modele2

prix_predit <- predict(modele2, example)
prix_predit
# L'estimation n'est pas du tout robuste, quand on supprime une donnée on obtient des coefs très différents et un prix predit très différent

#6
summary(modele)
modele3 <- lm(PRIX~CYL+PUIS+LON+LAR, data=vehicules)
summary(modele3)


modele4 <- lm(PRIX~CYL+PUIS+LON+LAR+VITESSE, data=vehicules)
summary(modele4)

# 7 

indices_app <- sort(sample(18, 12))
indices_test <- setdiff(1:18, indices_app)

data_train<- vehicules[indices_app,]
data_test <- vehicules[indices_test,]

modele4b <-  lm(PRIX~CYL+PUIS+LON+LAR+VITESSE, data=data_train)
summary(modele4b)

previsions <- predict(modele4b, data_test)
var(modele4b$residuals)
var(data_test$PRIX-previsions)
# variance plus élevée : logique, modele meilleur sur données d'apprentissage

plot(data_test$PRIX, previsions)
