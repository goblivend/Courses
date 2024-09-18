
library(corrplot)
library(FactoMineR)

setwd("D:/gits/Courses/ing2/PBS/stats")
# 1
library(MASS)

# 2
data(iris)
summary(iris)

# 3
iris.lda <- lda(Species~.,iris)
summary(iris.lda)
iris.lda

# 4
iris.lda$prior
iris.lda$counts
iris.lda$means
iris.lda$N
iris.lda$scaling

# 5
iris.num <- iris[, 1:4]

iris.predicted <- predict(iris.lda, iris.num)
summary(iris.predicted)

# 6
table(iris$Species, iris.predicted$class)

# 7
iris.predicted$posterior

# 8 
iris.Proba.Max <- apply(iris.predicted$posterior, 1, FUN=which.max)
table(iris.predicted$class, iris.Proba.Max)

# 9

ex <- iris[50,]
ex$Sepal.Length <- 5
ex$Sepal.Width <- 3.5
ex$Petal.Length <- 5
ex$Petal.Width <- 1

predict(iris.lda, ex)
# 9 bis : split avec données train et test

idx.train <- sort(sample(150, 100))
idx.test <- setdiff(1:150,idx.train)

iris.train <- iris[idx.train,]
iris.test <- iris[idx.test,]

iris.train.lda <- lda(Species~.,iris.train)

iris.test.predicted <- predict(iris.train.lda, iris.test[, 1:4])

table(iris.test$Species, iris.test.predicted$class)


# 10
iris.PCA<-PCA(iris,quali.sup=5)

iris.HCPC<-HCPC(iris.PCA)
iris.clust<-iris.HCPC$data.clust$clust
table(iris$Species, iris.clust)

# 11

vins <- read.csv2("vins.csv")
vins.lda <- lda(Label~.,vins)
vins.predict <- predict(vins.lda, vins)
table(vins$Label, vins.predict$class)


vins2 <- read.csv2("wine_italy.csv")
vins2.lda <- lda(class~.,vins2)
vins2.predict <- predict(vins2.lda, vins2)
table(vins2$class, vins2.predict$class)


vehicules <- read.csv2("Vehicules_TD_2023.csv")
vehicules.lda <- lda(GAMME~.,vehicules)
vehicules.predict <- predict(vehicules.lda, vehicules)
table(vehicules$GAMME, vehicules.predict$class)
