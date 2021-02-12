# UART frame

There was still the problem of how to distinguish between the commands of the
control module and the responses of the cell modules. Dividing 8-bit data bytes
into two messages is not desirable, so communication with nine data bits,
odd parity, and one stop bit (shorter 9O1) is used. The ninth data bit allows
signaling whether the message is a command or a response. The modules must
execute the commands and only forward the responses. This mode of communication
has been proven to be very reliable. The module always receives all message
inside the interrupt routine. In case the received message is a command it needs
to be saved and process later outside the interrupt. Otherwise, when you receive
a reply from some other cell module, you just forward it. The 9O1 configuration
is not common and is not supported by all devices. This can significantly limit
the ability to use any processor in the control modules, which is not desirable.
With a few smaller software solutions, we achieved that UART communication from
the outside is visible as eight data bits, even parity and two stop bits
(shorter 8E2). This is a much more widespread communication configuration and
most processors should support it. 9O1 is still used inside the system, only on
the outside it behaves like 8E2. This could only be achieved because 9O1 and 8E2
messages are the same length and have opposite parity. The picture shows us the
structure of the messages in more detail. When message is received from 8E2 to
9O1, the parity bit becomes the ninth data bit. On the other hand when
transmitting from 9O1 to 8E2, the ninth data bit becomes a parity bit. The
downside is that this bit needs to be calculated in software, which takes some
time in the interrupt routine, but it is feasible. Most likely, this kind of use
of UART communication is not an example of good practice, but in this particular
case it works great. The main advantage is that the user does not have to split
8-bit data bytes into two separate messages and modules can still distinguish
commands from responses.

![UART frame](https://raw.githubusercontent.com/Hrastovc/CarettaBMS/gh-pages/images/UARTframe.png)

The module always accepts the command and executes it first with an appropriate
response and only then forwards the command to the next module. This ensures an
easier system synchronization. In this way, we avoid race condition when the
first module would already start transmitting a response and the next module is
still executing the command. Interrupts should ensure this still works, but it
would be a weak point of communication that can lead to CPU overload due to
nesting interrupts.
