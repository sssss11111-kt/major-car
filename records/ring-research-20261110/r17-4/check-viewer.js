(async () => {
  const data = JSON.parse(document.getElementById('data').textContent);
  const check = (value, message) => { if (!value) throw new Error(message); };
  check(data.length === 128, 'frame count');
  check(data.every((frame, i) => frame.index === i + 120), 'original numbering');
  check(document.getElementById('slider').min === '120', 'minimum');
  check(document.getElementById('slider').max === '247', 'maximum');
  for (let i = 0; i < data.length; i++) {
    show(i);
    for (let wait = 0; document.body.dataset.loadedFrame !== String(i + 120); wait++) {
      check(wait < 100, `image load ${i + 120}`);
      await new Promise(resolve => setTimeout(resolve, 10));
    }
    for (const id of ['gray', 'old', 'current', 'exit']) {
      const pixels = document.getElementById(id).getContext('2d').getImageData(0, 0, 188, 120).data;
      check(pixels.some((value, offset) => offset % 4 === 3 && value === 255), `canvas ${id}`);
    }
  }
  document.getElementById('next').click();
  check(document.getElementById('jump').value === '247', 'upper bound');
  show(0);
  document.getElementById('prev').click();
  check(document.getElementById('jump').value === '120', 'lower bound');
  document.getElementById('reject').click();
  check(document.getElementById('jump').value === '156', 'next rejection');
  show(0);
  document.getElementById('play').click();
  await new Promise(resolve => setTimeout(resolve, 400));
  document.getElementById('play').click();
  check(Number(document.getElementById('jump').value) > 120, 'playback');
  document.getElementById('jump').value = '203';
  document.getElementById('jump').dispatchEvent(new Event('change'));
  await new Promise(resolve => setTimeout(resolve, 100));
  check(document.body.dataset.loadedFrame === '203', 'jump to original frame');
  return {passed: true, checked_frames: 128, loaded_canvases: 512, minimum: 120, maximum: 247,
    next_rejection: 156, playback: true, selected_frame: 203};
})()
