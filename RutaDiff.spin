CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
                                                                                                        
OBJ
  s2    : "S2"
  term  : "FullDuplexSerial"

PUB start | comando, valor, valor_izq, valor_der, tiempo_ms

  s2.start
  s2.start_motors
  s2.set_speed(2)
  s2.set_volume(50)
  
  term.start(31, 30, 0, 9600) 
  s2.beep
  term.tx("K") 
  s2.set_led(3, 1)

  repeat
    comando := term.rx
    
    case comando
      "F":
        valor := LeerNumero
        s2.set_led(3, 0)
        s2.go_forward(valor)
        repeat until s2.move_ready
        s2.set_led(3, 1)
        term.tx("K")      
        
      "T":
        valor := LeerNumero
        s2.set_led(3, 0)
        s2.turn_by(valor)
        repeat until s2.move_ready
        s2.set_led(3, 1)
        term.tx("K")
        
      "M":
        valor_izq := LeerNumero
        valor_der := LeerNumero
        tiempo_ms := LeerNumero
        
        s2.set_led(3, 0)
        s2.wheels_now(valor_izq, valor_der, 0)
        waitcnt((clkfreq / 1000) * tiempo_ms + cnt)
        s2.stop_now
        s2.set_led(3, 1)
        term.tx("K")
        
      "V":  ' <--- NUEVO COMANDO DE VELOCIDAD CONTINUA
        valor_izq := LeerNumero
        valor_der := LeerNumero
        
        s2.set_led(3, 0)
        
        ' 1. Aplica velocidad. El tercer parámetro "0" le indica a la librería que no hay timeout.
        s2.wheels_now(valor_izq, valor_der, 0) 
        
        s2.set_led(3, 1)
        ' 2. Responde Inmediatamente sin frenar.
        term.tx("K")
        
      "B":
        valor := LeerNumero
        s2.beep
        term.tx("K")

PRI LeerNumero : valor | caracter, signo
  valor := 0
  signo := 1
  repeat
    caracter := term.rx
    case caracter
      "-": 
        signo := -1
      "0".."9": 
        valor := (valor * 10) + (caracter - "0")
      ",": 
        quit
  return valor * signo