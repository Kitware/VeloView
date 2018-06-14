define print_packet_csv
set $p = $arg0
set $f = 0
while $f < 12
    set $dsr = 0
    printf "blkIden:%x,f_az:%d",$p.firingData[$f].blockIdentifier,$p.firingData[$f].rotationalPosition
    while $dsr < 32
        printf "%d,%d",$p.firingData[$f].laserReturns[$dsr].distance, $p.firingData[$f].laserReturns[$dsr].intensity
        set $dsr = $dsr +1
    end
    set $f = $f +1
end
printf "gps:%d",$p.gpsTimestamp
p/x $p.factoryField1
p/x $p.factoryField2
