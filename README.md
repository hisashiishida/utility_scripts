# Utility scripts


## List of scripts
- republish_reference.py : republish measured_cp with refence frame set.
- 


## How to use
### Republishing the rostopics
Use the following command to run the model
*--ns1*: old_namespace, *--ns2*: new_namespace, *-t/--topics*: topic names, 
*--ref*: refernce 



```bash
python3 republish_reference.py --ns1 /old --ns2 /new -t topic --ref reference

```

## ToDo
### Republishing the rostopics
- Check wheather the topic is just receved or not