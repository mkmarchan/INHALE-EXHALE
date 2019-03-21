(
s.options.numOutputBusChannels_(4);
s.waitForBoot({
	var synth1, synth2, note, decoder;

	decoder = FoaDecoderMatrix.newQuad(30 * pi / 180, 'energy');

	synth1 = CtkSynthDef.new(\testSynth, {
		var numSpeakers = 4;
		var signal;
		var out;


		signal = PinkNoise.ar(0.5);

		out = PanAz.ar(numSpeakers, signal, LFSaw.ar(0.1), 1, 2, 0.5);

		Out.ar(0, out)});

	synth2 = CtkSynthDef.new(\testSynth2, {
		var signal;
		var out;

		signal = PinkNoise.ar(0.5);

		out = FoaPanB.ar(signal, LFSaw.ar(0.1, 0, pi));

		out = FoaDecode.ar(out, decoder);

		Out.ar(0, out)});

	synth2.note.play


});

)