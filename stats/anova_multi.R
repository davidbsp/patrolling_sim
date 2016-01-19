#
# How to use:
#
# $ Rscript anova_multi.R <file_idleness_1> <file_idleness_2> ... <file_idleness_n>
#
# Example:
#
# $ Rscript anova_multi.R results/grid_4/SEBS/iocchi-d1/20151117_010310_idleness.csv results/grid_4/SEBS/iocchi-d1/20151117_020344_idleness.csv


idleness_aov <- function(f1, f2) {

    a <- read.csv(f1,sep=";",header=TRUE)
    b <- read.csv(f2,sep=";",header=TRUE)

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

}



args <- commandArgs(trailingOnly = TRUE)

n <- length(args)
a <- matrix(ncol=n, nrow=n)

#print(n)

i <- 1
while (i <= n) {
  j <- 1
  while (j <= n) {
    #print(args[i])
    #print(args[j])
    if (j>i) {
      a[i,j] <- idleness_aov(args[i], args[j])
      #print(a[i,j])
      cat(sprintf("%.3f ",a[i,j]))
    }
    else {
      cat("      ")
    }
    if (j<n) {
      cat("; ")
    }
    j <- j+1
  }
  i <- i+1
  cat("\n")
}

#print(a)





