#
# How to use:
#
# $ Rscript boxplots.R <title> <file_idleness_1> <label_1> ... <file_idleness_n> <label_n>
#
# Example:
#
# $ Rscript boxplots.R grid_4 results/grid_4/SEBS/iocchi-d1/20151117_010310_idleness.csv ... 


boxplots_idl <- function(v) {

    plot_title = v[1]
    
    n <- (length(v)-1)/2
    z <- vector(length=n)
    means <- vector(length=n)
    l <- vector(length=n)

    i <- 1
    j <- 2
    while (i <= n) {

#print(vf[i*2-1])

        m <- read.csv(v[j],sep=";",header=TRUE)      # a[,4] idleness values vector for dataset a
        l[i] <- v[j+1]

#print(l[i])

        vname = sprintf("a%d",i)

#print(vname)
        assign(vname,m[,4])

        means[i] = mean(get(vname))

        z[i] = vname

        i <- i+1
        j <- j+2
 
    }

    dd <- lapply(z, get, envir=environment())
    names(dd) <- l
    
    boxplot(dd,outline=TRUE,horizontal=FALSE, range=1.5, las=2, main=plot_title,  xlab="", ylab="Idleness (s)")

    points(means, pch = 8, cex = 1.25)  #3=+, 8=*

}



args <- commandArgs(trailingOnly = TRUE)

boxplots_idl(args)


