#
# How to use:
#
# $ Rscript plots.R <title> <file_idleness_1> <label_1> ... <file_idleness_n> <label_n>
#
# Example:
#
# $ Rscript plots.R grid_4 results/grid_4/SEBS/iocchi-d1/20151117_010310_idleness.csv ... 


plots_idl <- function(v) {
    
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

        #means[i] = mean(get(vname))

        #z[i] = vname

        i <- i+1
        j <- j+2
 
        plot(m[,1], m[,4], type="p", cex=0.5, main=plot_title, xlab="Time (s)", ylab="Idleness (s)", ylim=c(0,1000))

    }

}



args <- commandArgs(trailingOnly = TRUE)

plots_idl(args)


