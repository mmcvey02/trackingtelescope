import datetime
import io
import json
import os
import tarfile
import tempfile
import unittest
import urllib.error
import urllib.parse

from roomba_mapper import cloud

BLID = "0B9B38BB1914433B"


def make_bundle(files):
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as tar:
        for name, doc in files.items():
            data = json.dumps(doc).encode()
            info = tarfile.TarInfo(f"bundle/{name}.geojson")
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
    return buf.getvalue()


BUNDLE = make_bundle({
    "manifest": {"features": [{"type": "rooms"}, {"type": "trajectories"}]},
    "rooms": {"type": "FeatureCollection", "features": [
        {"type": "Feature", "id": "1", "properties": {"name": "Kitchen"},
         "geometry": {"type": "Polygon", "coordinates": [[[0, 0], [4, 0], [4, 3], [0, 3], [0, 0]]]}}]},
    "trajectories": {"type": "FeatureCollection", "features": [
        {"type": "Feature", "properties": {"index": 0},
         "geometry": {"type": "LineString", "coordinates": [[0.2, 0.2], [3.8, 0.2], [3.8, 0.5]]}}]},
})


class Resp(io.BytesIO):
    def __init__(self, data, status=200):
        super().__init__(data)
        self.status = status

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


class FakeIRobot:
    """Answers like iRobot's discovery, login and REST services."""

    def __init__(self, deny=(), run_map_at=None):
        self.deny = deny
        self.run_map_at = run_map_at
        self.requests = []

    def __call__(self, req, timeout=None):
        url = req.full_url
        self.requests.append(req)
        js = lambda obj: Resp(json.dumps(obj).encode())
        if "discover/endpoints" in url:
            return js({"gigya": {"api_key": "K", "datacenter_domain": "us1.gigya.com"},
                       "current_deployment": "v011",
                       "deployments": {
                           "v011": {"httpBase": "https://unauth3.example", "httpBaseAuth": "https://auth3.example"},
                           "v007": {"httpBase": "https://unauth2.example", "httpBaseAuth": "https://auth2.example"}}})
        if "accounts.login" in url:
            return js({"UID": "u1", "UIDSignature": "s", "signatureTimestamp": "1"})
        if url.endswith("/v2/login"):
            return js({"robots": {BLID: {"name": "Silvanus", "sku": "Y014020", "svcDeplId": "v007",
                                         "password": "robot-secret", "softwareVer": "congo+1.1.22"}},
                       "credentials": {"AccessKeyId": "AKIA", "SecretKey": "sk", "SessionToken": "tok",
                                       "CognitoId": "us-east-1:abc"}})
        if url.startswith("https://download.example/"):
            return Resp(BUNDLE)
        # signed REST calls
        assert url.startswith("https://auth2.example/"), url  # the robot's own deployment
        headers = {k.lower(): v for k, v in req.header_items()}
        assert headers["authorization"].startswith("AWS4-HMAC-SHA256 Credential=AKIA/")
        assert "/us-east-1/execute-api/aws4_request" in headers["authorization"]
        assert headers["x-amz-security-token"] == "tok"
        path = urllib.parse.urlsplit(url).path
        if any(d in path for d in self.deny):
            raise urllib.error.HTTPError(url, 403, "Forbidden", {}, io.BytesIO(b'{"message":"denied"}'))
        if path == "/v1/p2maps":
            return js([{"p2map_id": "M1", "active_p2mapv_id": "V1", "name": "Home", "state": "active"}])
        if path == "/v1/p2maps/M1":
            return js({"p2map_id": "M1", "active_p2mapv_id": "V1"})
        if path == "/v1/p2maps/M1/versions/V1":
            return js({"geojson_details": {"regions": [{"id": "1", "name": "Kitchen"}]}})
        if path == "/v1/p2maps/M1/versions/V1/geojson":
            return js({"map_url": "https://download.example/bundle.tgz?sig=xyz"})
        if path == f"/v1/{BLID}/missionhistory":
            return js([{"missionId": "01ABC", "durationM": 31, "sqft": 220,
                        "v4maps": ["RUNMAP1"], "v4maps_uploadfmt": "v4odm1map"}])
        if path == f"/v1/robots/{BLID}/timeline":
            return js({"events": []})
        if path == "/v1/p2maps/RUNMAP1" and self.run_map_at == "p2map":
            return js({"map_url": "https://download.example/run.tgz?sig=abc"})
        raise urllib.error.HTTPError(url, 404, "Not Found", {}, io.BytesIO(b"{}"))


class SigV4Tests(unittest.TestCase):
    def test_aws_reference_vector(self):
        # AWS SigV4 test suite "get-vanilla"
        now = datetime.datetime(2015, 8, 30, 12, 36, tzinfo=datetime.timezone.utc)
        h = cloud.sigv4_sign("GET", "https://example.amazonaws.com/", "AKIDEXAMPLE",
                             "wJalrXUtnFEMI/K7MDENG+bPxRfiCYEXAMPLEKEY", "us-east-1", "service", now=now)
        self.assertTrue(h["authorization"].endswith(
            "Signature=5fa00fa31553b73ebf1942676e86291e8372ff2a2260956d9b8aae1d763fbf31"))
        self.assertIn("SignedHeaders=host;x-amz-date", h["authorization"])

    def test_query_is_sorted_and_encoded(self):
        now = datetime.datetime(2015, 8, 30, 12, 36, tzinfo=datetime.timezone.utc)
        a = cloud.sigv4_sign("GET", "https://h/p?b=2&a=1%2C2", "K", "S", "us-east-1", "x", now=now)
        b = cloud.sigv4_sign("GET", "https://h/p?a=1%2C2&b=2", "K", "S", "us-east-1", "x", now=now)
        self.assertEqual(a["authorization"], b["authorization"])


class BundleTests(unittest.TestCase):
    def test_unpack_and_summarize(self):
        files = cloud.unpack_bundle(BUNDLE)
        self.assertEqual(set(files), {"manifest", "rooms", "trajectories"})
        s = cloud.summarize_geojson(files["trajectories"])
        self.assertEqual(s["features"], 1)
        self.assertEqual(s["geometry"], {"LineString": 1})
        self.assertEqual(s["x_range"], [0.2, 3.8])
        self.assertIsNone(cloud.summarize_geojson(files["manifest"]))

    def test_redaction(self):
        out = cloud.redact({"password": "p", "credentials": {"a": 1}, "map_url": "u",
                            "robots": {"X": {"name": "n", "password": "p"}}, "sqft": 3})
        self.assertEqual(out["password"], "<redacted>")
        self.assertEqual(out["credentials"], "<redacted>")
        self.assertEqual(out["map_url"], "<redacted>")
        self.assertEqual(out["robots"]["X"]["password"], "<redacted>")
        self.assertEqual(out["sqft"], 3)


class CloudProbeTests(unittest.TestCase):
    def test_full_probe_downloads_maps_and_history(self):
        fake = FakeIRobot()
        lines = []
        with tempfile.TemporaryDirectory() as d:
            report = cloud.cloud_probe("me@example.com", "pw", out_dir=d, log=lines.append, opener=fake)
            self.assertEqual(report["endpoints"]["maps (p2maps)"], 200)
            self.assertEqual(report["endpoints"]["mission history"], 200)
            self.assertEqual(report["features"], {"rooms": 1, "trajectories": 1})
            self.assertTrue(os.path.exists(os.path.join(d, "map_M1", "trajectories.json")))
            saved = ""
            for root, _dirs, names in os.walk(d):
                for n in names:
                    with open(os.path.join(root, n)) as fh:
                        saved += fh.read()
        for secret in ("robot-secret", "AKIA", "tok", "xyz", "me@example.com"):
            self.assertNotIn(secret, saved)
        self.assertTrue(any("paths/coverage" in line for line in lines))
        # the account password only went to the login service
        sent_pw = [r.full_url for r in fake.requests if r.data and b"pw" in r.data]
        self.assertEqual(len(sent_pw), 1)
        self.assertIn("accounts.login", sent_pw[0])

    def test_per_run_map_search(self):
        fake = FakeIRobot(deny=("/v1/p2maps",), run_map_at="p2map")
        fake.deny = ("/v1/p2maps/livemap",)  # p2maps list allowed but empty below
        lines = []
        with tempfile.TemporaryDirectory() as d:
            report = cloud.cloud_probe("me@example.com", "pw", out_dir=d, log=lines.append, opener=fake)
            self.assertTrue(os.path.exists(os.path.join(d, "mission_map", "p2map_by_id_map_url.bin")))
        tried = report["mission_map"]["tried"]
        self.assertEqual(tried["p2map by id"], 200)
        self.assertEqual(tried["mission by id"], 404)
        self.assertEqual(tried["live map"], 403)
        self.assertEqual(report["mission_map"]["map_id_format"], "v4odm1map")
        self.assertIn("trajectories", report["features"])
        self.assertTrue(any("paths/coverage" in line for line in lines))

    def test_empty_answers_are_not_counted_as_maps(self):
        fake = FakeIRobot()
        lines = []
        with tempfile.TemporaryDirectory() as d:
            report = cloud.cloud_probe("me@example.com", "pw", out_dir=d, log=lines.append, opener=fake)
        self.assertEqual(report["mission_map"]["tried"]["robot timeline"], 204)
        self.assertFalse(any("Some per-run map requests were answered" in line for line in lines))

    def test_unknown_download_format_is_identified(self):
        self.assertEqual(cloud.describe_bytes(b"\x1f\x8b\x08\x00"), "gzip")
        self.assertEqual(cloud.describe_bytes(b"\x78\x9cabc"), "zlib")
        self.assertEqual(cloud.describe_bytes(b"\x0a\x05hello"), "unknown binary (protobuf?)")

    def test_denied_requests_are_reported(self):
        fake = FakeIRobot(deny=("/v1/p2maps", "missionhistory", "pmaps", "/v1/robots"))
        lines = []
        with tempfile.TemporaryDirectory() as d:
            report = cloud.cloud_probe("me@example.com", "pw", out_dir=d, log=lines.append, opener=fake)
        self.assertEqual(report["endpoints"]["maps (p2maps)"], 403)
        self.assertEqual(report["maps"], [])
        self.assertTrue(any("refused every request" in line for line in lines))

    def test_login_failure(self):
        class BadLogin(FakeIRobot):
            def __call__(self, req, timeout=None):
                if "accounts.login" in req.full_url:
                    return Resp(json.dumps({"errorCode": 403042, "errorDetails": "invalid loginID or password"}).encode())
                return super().__call__(req, timeout)
        with self.assertRaisesRegex(cloud.CloudError, "invalid loginID or password"):
            cloud.IRobotCloud("a", "b", opener=BadLogin()).login()


if __name__ == "__main__":
    unittest.main()


class ErrorReasonTests(unittest.TestCase):
    def test_reasons_are_short_and_anonymous(self):
        from roomba_mapper.cloud import error_reason
        self.assertEqual(error_reason({"message": "Missing Authentication Token"}),
                         "'Missing Authentication Token'")
        text = error_reason({"Message": "User: arn:aws:sts::123456789012:assumed-role/a/b is not authorized "
                                        "on resource: arn:aws:execute-api:us-east-1:123456789012:x/GET/v1"})
        self.assertIn("is not authorized", text)
        self.assertNotIn("123456789012", text)
        self.assertEqual(error_reason(b"<html>"), "")
