
ip=$(nmap -sn 192.168.1.0/24 | sed -r -n -e 's/^.+raspberry.+\((.+)\)$/\1/p')

echo "IdefX ip is $ip"
echo $ip > build/IdefX_ip.txt
echo "Connect using : pi@$ip"