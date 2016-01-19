#
# How to use:
#
# $ Rscript boxplots.R <file_idleness_1> ... <file_idleness_n>
#
# Example:
#
# $ Rscript boxplots.R results/grid_4/SEBS/iocchi-d1/20151117_010310_idleness.csv ... 


boxplots_idl <- function(vf) {

    n <- length(vf)/2
    z <- vector(length=n)
    means <- vector(length=n)
    l <- vector(length=n)

    i <- 1
	while (i <= n) {

#print(vf[i*2-1])

    	m <- read.csv(vf[i*2-1],sep=";",header=FALSE)      # a[,4] idleness values vector for dataset a
        l[i] <- vf[i*2]

#print(l[i])

        vname = sprintf("a%d",i)

#print(vname)
    	assign(vname,m[,4])

    	means[i] = mean(get(vname))

 		z[i] = vname

		i <- i+1
 
	}

    dd <- lapply(z, get, envir=environment())
    names(dd) <- l
    
    boxplot(dd,outline=TRUE,horizontal=FALSE,range=3,las=2)
#    boxplot(dd,outline=FALSE,horizontal=FALSE,range=3,las=2)

	points(means, pch = 8, cex = 1.5)

}



args <- commandArgs(trailingOnly = TRUE)

boxplots_idl(args)


