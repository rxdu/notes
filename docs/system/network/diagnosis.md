# Network Diagnosis

## Commands

* **nslookup**: to perform a DNS lookup

    ```bash
    $ nslookup google.com    
    ```
* **netstat**: to get statistics for network ports and shows port availability

    ```bash
    # to list all TCP ports
    $ netstat -at
    ```
* **traceroute**: to track the route that packets take to reach a destination on a TCP/IP network

    ```bash
    $ sudo traceroute -T www.google.com
    ```
    
