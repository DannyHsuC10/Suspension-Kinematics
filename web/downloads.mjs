const encoder = new TextEncoder();

export function download(name, contents, type) {
  const url = URL.createObjectURL(new Blob([contents], { type }));
  const link = document.createElement('a');
  link.href = url;
  link.download = name;
  document.body.append(link);
  link.click();
  link.remove();
  setTimeout(() => URL.revokeObjectURL(url), 2000);
}

function crc32(bytes) {
  let crc = -1;
  for (const value of bytes) {
    crc ^= value;
    for (let bit = 0; bit < 8; bit++) crc = (crc >>> 1) ^ (crc & 1 ? 0xedb88320 : 0);
  }
  return (crc ^ -1) >>> 0;
}

// Store each CSV unchanged in a standard ZIP archive.
export function zipCsv(files) {
  const entries = files.map(({ name, text }) => ({ name: encoder.encode(name), data: encoder.encode(text) }));
  const contentSize = entries.reduce((sum, { name, data }) => sum + 30 + name.length + data.length, 0);
  const directorySize = entries.reduce((sum, { name }) => sum + 46 + name.length, 0);
  const bytes = new Uint8Array(contentSize + directorySize + 22);
  const view = new DataView(bytes.buffer);
  const u16 = (position, value) => view.setUint16(position, value, true);
  const u32 = (position, value) => view.setUint32(position, value, true);
  let local = 0;
  let directory = contentSize;
  for (const { name, data } of entries) {
    const crc = crc32(data);
    u32(local, 0x04034b50);
    u16(local + 4, 20);
    u16(local + 6, 0x800);
    u16(local + 12, 33);
    u32(local + 14, crc);
    u32(local + 18, data.length);
    u32(local + 22, data.length);
    u16(local + 26, name.length);
    bytes.set(name, local + 30);
    bytes.set(data, local + 30 + name.length);
    u32(directory, 0x02014b50);
    u16(directory + 4, 20);
    u16(directory + 6, 20);
    u16(directory + 8, 0x800);
    u16(directory + 14, 33);
    u32(directory + 16, crc);
    u32(directory + 20, data.length);
    u32(directory + 24, data.length);
    u16(directory + 28, name.length);
    u32(directory + 42, local);
    bytes.set(name, directory + 46);
    local += 30 + name.length + data.length;
    directory += 46 + name.length;
  }
  u32(directory, 0x06054b50);
  u16(directory + 8, entries.length);
  u16(directory + 10, entries.length);
  u32(directory + 12, directorySize);
  u32(directory + 16, contentSize);
  return bytes;
}
