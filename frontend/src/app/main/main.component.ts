import {ChangeDetectorRef, Component, OnInit} from '@angular/core';

@Component({
  selector: 'app-main',
  templateUrl: './main.component.html',
  styleUrls: ['./main.component.css']
})
export class MainComponent implements OnInit {

  ws: WebSocket;
  recognition: SpeechRecognition;
  result: string;
  isRecording: boolean = false;
  commandsIncorrect: boolean = undefined;
  executingCommands: boolean = false;
  executionResult: string;

  constructor(private changeDetectorRef: ChangeDetectorRef) {
  }


  ngOnInit(): void {
    this.initRecognition();
  }

  startRecording() {
    this.executionResult = undefined;
    this.isRecording = true;
    this.changeDetectorRef.detectChanges();
    this.recognition.start();
  }

  private initRecognition() {
    let typ = typeof SpeechRecognition;
    // @ts-ignore
    this.recognition = typ === "undefined" ? new webkitSpeechRecognition() :
      new SpeechRecognition();
    this.recognition.continuous = false;
    this.recognition.lang = 'it-IT';
    this.recognition.interimResults = false;
    this.recognition.maxAlternatives = 1;
    this.initRecognitionCallbacks();
  }

  private initRecognitionCallbacks() {
    this.recognition.onspeechend = () => {
      this.isRecording = false;
      this.recognition.stop();
    };
    this.recognition.onnomatch = () => {
      this.result = 'Non sono riuscito a trascrivere le parole';
      this.changeDetectorRef.detectChanges();
    };
    this.recognition.onerror = (event) => {
      this.result = 'Errore nel riconoscimento';
      this.changeDetectorRef.detectChanges();
    };
    this.recognition.onresult = (event) => {
      this.result = `${event.results[0][0].transcript}`;
      const commands: string[] = this.result.split(" ");
      commands.every(singleCommand => {
        const isCommandCorrect: boolean = singleCommand.toLowerCase() === 'avanti' || singleCommand.toLowerCase() === 'destra'
          || singleCommand.toLowerCase() === 'sinistra'
        if (!isCommandCorrect) {
          // console.log('Entro qui');
          this.commandsIncorrect = true;
          return false;
        }
        this.commandsIncorrect = false;
        return true;
      });
      this.changeDetectorRef.detectChanges();
    }
  }

  resetRecording() {
    this.commandsIncorrect = undefined;
    this.startRecording();
  }

  sendData() {
    this.executingCommands = true;
    this.changeDetectorRef.detectChanges()
    this.ws = new WebSocket('ws://localhost:8090');
    this.ws.onopen = () => this.handleAfterConnection();
    this.ws.onmessage = (event) => {
      this.executingCommands = false;
      // console.log(event);
      // console.log('Dati: ' + event.data);
      this.executionResult = event.data;
      this.changeDetectorRef.detectChanges();
    };
  }

  handleAfterConnection() {
    // console.log('Connesso!');
    this.ws.send(this.result.toLowerCase());
  }

  stopRecording() {
    this.isRecording = false;
    this.changeDetectorRef.detectChanges();
    this.recognition.stop();
  }
}
