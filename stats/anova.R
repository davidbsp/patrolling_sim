#
# How to use:
#
# $ Rscript anova.R <file_idleness_1> <file_idleness_2>
#
# Example:
#
# $ Rscript anova.R results/grid_4/SEBS/iocchi-d1/20151117_010310_idleness.csv results/grid_4/SEBS/iocchi-d1/20151117_020344_idleness.csv

args <- commandArgs(trailingOnly = TRUE)

f1 = args[1]
f2 = args[2]

a <- read.csv(f1,sep=";")
b <- read.csv(f2,sep=";")

# a[,4] idleness values vector for dataset a
# b[,4] idleness values vector for dataset b

# combined idleness vector
v = c(a[,4],b[,4])

# labels vector
sa = rep('A',times=length(a[,4]))
sb = rep('B',times=length(b[,4]))
s = c(sa,sb)

# create data frame
dd = data.frame(s,v)

# change the column names
colnames(dd) <- c("Run", "Idleness")

# verify the data frame
#head(dd)

#compute ANOVA statistics
x = aov(Idleness ~ Run, dd)

#display the result
summary(x)

summary(x)[[1]][[1,"Pr(>F)"]]


