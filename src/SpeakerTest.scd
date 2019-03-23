(
s.options.numOutputBusChannels_(4);
s.waitForBoot({
	var synth1, synth2, synth3, note, encoder, decoder;

	decoder = FoaDecoderMatrix.newQuad(30 * pi / 180, 'energy');

	synth1 = CtkSynthDef.new(\testSynth, {
		var numSpeakers = 4;
		var signal;
		var out;


		signal = PinkNoise.ar(0.5);

		out = PanAz.ar(numSpeakers, signal, LFSaw.ar(0.1));

		Out.ar(0, out)});

	synth2 = CtkSynthDef.new(\testSynth2, {
		var signal;
		var out;

		signal = PinkNoise.ar(0.5);
		signal = signal * LFPulse.ar(0.2, [0, 0.8, 0.6, 0.4], 0.2);

		Out.ar(0, signal)});


	synth3 = CtkSynthDef.new(\testSynth3, {
		var signal;
		var out;


		signal = PinkNoise.ar(0.5);

		out = FoaPanB.ar(signal, LFSaw.ar(0.25, 0, pi));

		Out.ar(0, out)});

	synth2.note.play


});

)