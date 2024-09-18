
library(corrplot)
library(FactoMineR)

# 1
setwd("D:/gits/Courses/ing2/PBS/stats")

data <- read.csv2("vins.csv", row.names=1)

# 2
summary(data)

# 3
fix(data)

# 4
Matrix <- cor(data[,-c(1,2)])
plot(data[,29])
corrplot(Matrix)

# 5 
data.PCA <- PCA(data, quali.sup=c(1,2))

vars.exp <- data.PCA$eig[,1]

# 6 
sum(vars.exp)
barplot(vars.exp)

# 7
data.coords <- data.PCA$var$coord
sqsum <- function (v) sum(v^2)
apply(data.coords,1, sqsum)

#8
data.ind <- data.PCA$ind$coord

#9 

I.Saumur<-which(data$Label=="Saumur")
I.Bourgueuil<-which(data$Label=="Bourgueuil")
I.Chinon<-which(data$Label=="Chinon")

dev.new()
plot(data.ind[,1:2])
points(data.ind[I.Saumur,1:2],pch=19,col="red")
points(data.ind[I.Chinon,1:2],pch=19,col="blue")
points(data.ind[I.Bourgueuil,1:2],pch=19,col="cyan")

#10 : flemmmmmmmmeeeeeeeee

data2 <- read.csv2("wine_italy.csv")
summary(data2)
corrplot(cor(data2[,-1]))

# 11
data2.pca <- PCA(data2, quali.sup = 1)

data2.IND<-data2.pca$ind$coord
# 13

I.1<-which(data2$class==1)
I.2<-which(data2$class==2)
I.3<-which(data2$class==3)

dev.new()
plot(data2.IND[,1:2])
points(data2.IND[I.1,1:2],pch=19,col="red")
points(data2.IND[I.2,1:2],pch=19,col="blue")
points(data2.IND[I.3,1:2],pch=19,col="limegreen")

#14
data2.HCPC<-HCPC(data2.pca)

summary(data2.HCPC)
summary(data2.HCPC$data.clust)

#16 

data2.HCPC$data.clust$clust

#17

classe <- data2$class
clust <- data2.HCPC$data.clust$clust
table(classe, clust)
