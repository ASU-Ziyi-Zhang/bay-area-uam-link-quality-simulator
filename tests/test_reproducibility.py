from pathlib import Path

from scripts.verify_reference import sha256


def test_text_hash_is_independent_of_platform_line_endings(tmp_path: Path) -> None:
    lf = tmp_path / "lf.csv"
    crlf = tmp_path / "crlf.csv"
    lf.write_bytes(b"site,value\nBS01,1\n")
    crlf.write_bytes(b"site,value\r\nBS01,1\r\n")
    assert sha256(lf) == sha256(crlf)


def test_binary_hash_preserves_raw_bytes(tmp_path: Path) -> None:
    first = tmp_path / "first.bin"
    second = tmp_path / "second.bin"
    first.write_bytes(b"a\r\nb")
    second.write_bytes(b"a\nb")
    assert sha256(first) != sha256(second)
